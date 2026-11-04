/**
 * @file LEMLink.cpp
 *
 * @brief Orbiter plugin that lets LEMBridge's bridge.py drive NASSP's LM panel.
 *
 * NASSP addresses every cockpit control by name - "ORDEALAltSetRotary",
 * "P01_EngineThrustCont_EngArm" - and LEMBridge's panels/catalog.json already stores those
 * names per control. This plugin is the piece that turns a name and a value on
 * a UDP socket into the switch actually moving.
 *
 * It is a SEPARATE DLL and does not patch NASSP. Project Apollo already
 * exposes a panel interface to outside modules, because its own MFD is a
 * separate DLL that uses it: `PanelConnector` on the vessel answers
 * MFD_PANEL_GET_ITEM_STATE and MFD_PANEL_SET_ITEM_STATE, and
 * `GetVesselConnector()` hands a pointer to it to anyone who asks. We are the
 * second client of a path that already works.
 *
 * The one hard constraint is in vendor/connector.cpp: reaching the vessel is a
 * static_cast to ProjectApolloConnectorVessel followed by a virtual call,
 * guarded only by a magic number. That is a shared C++ ABI, not a stable
 * interface, so this must be built against the NASSP revision that produced
 * the LEM.dll it will run beside. vendor/REVISION records which that is. A
 * mismatch compiles cleanly and corrupts memory at run time.
 *
 * Nothing here knows about the simpit's hardware. It moves named controls and
 * reports named controls; LEMBridge's config/mapping.csv on the PC still owns every
 * decision about which physical input means what. Same rule as LEMBridge's panel-node firmware.
 */
#define ORBITER_MODULE

/*
 * Winsock 2 before anything that reaches windows.h, which Orbitersdk.h does.
 * Plain windows.h pulls in winsock.h - Winsock 1 - and the two headers declare
 * the same structs, so the wrong order is forty redefinition errors deep in
 * the Windows SDK rather than anything that names this file.
 * WIN32_LEAN_AND_MEAN keeps winsock.h out from under us either way.
 */
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdarg.h>
#include <stdlib.h>

#include "Orbitersdk.h"
#include "connector.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 * The connector type nassp_patch/patch.md adds to NASSP's ConnectorType enum, for
 * LM displays that are not panel items - the caution and warning lights, the
 * timers, and meter readings. Declared here rather than included, so this
 * builds against stock NASSP: an unpatched LM simply registers no connector of
 * this type, GetConnector() returns NULL, and the display verbs answer
 * "! no-display".
 *
 * The value must match the patched enum. It is appended at the end there for
 * exactly this reason - an ordinal that moves would silently address the wrong
 * connector across the DLL boundary.
 */
#define LM_DISPLAY_DATA ((ConnectorType)(VHF_RNG + 1))

/*
 * Message numbers and reply layouts: the SAME header the patched LEM.dll is
 * built from. This file used to declare its own copy of the enum, which is two
 * definitions of one ABI that nothing checked against each other.
 */
#include "nassp_patch/lmdd_types.h"

/// Where LEMBridge's bridge.py sends commands. Not 8051 (Reentry) or 8052 (node events).
#define LEMLINK_PORT 8060

/*
 * NASSP's own LM PCM telemetry listener, not ours: LM_PCM binds INADDR_ANY on
 * this port in lm_telecom.cpp and streams the real Apollo downlink. The
 * overlay advertises it because a pit needs both endpoints and only one of
 * them is discoverable from the panel side.
 *
 * Deliberately NOT probed. That socket is `listen(s, 1)` with a single accept,
 * so a once-a-second connect to see whether it is up would take the slot the
 * bridge needs, or fight it for it. Show the address; let the bridge find out
 * by connecting for real.
 */
#define LEMLINK_TELEM_PORT 14243

/// Datagrams drained per frame. A bound, so a flood cannot stall the sim.
#define MAX_PACKETS_PER_STEP 32

#define RECV_BUF 1024

// ---------------------------------------------------------------------------
//                              PANEL CONNECTOR
// ---------------------------------------------------------------------------

/**
 * Our end of NASSP's MFD_PANEL_INTERFACE.
 *
 * Deliberately not vendored from src_mfd/MFDconnector.cpp: that file pulls in
 * checklistController.h and most of the checklist subsystem for features we do
 * not use. The two messages we need are three lines each, and the enum that
 * names them is already in connector.h.
 */
class PanelClient : public Connector
{
public:
    PanelClient() : peer(NULL), logged(false) { type = MFD_PANEL_INTERFACE; }

    /**
     * The vessel's panel connector, held WITHOUT calling ConnectTo().
     *
     * Connector is a one-to-one design - `connectedTo` is a single pointer and
     * ConnectTo() refuses an occupied peer - and the LM's panel connector is
     * already spoken for: NASSP's own ChecklistController takes it at
     * checklistController.cpp:518, during vessel init, before any plugin gets
     * a chance. Connecting properly is therefore not available to us and never
     * will be while the checklist controller exists.
     *
     * It is also not necessary. Connector::SendMessage does exactly one thing,
     * `connectedTo->ReceiveMessage(this, m)`, and PanelConnector::ReceiveMessage
     * ignores its `from` argument completely - it checks m.destination and
     * dispatches straight to PanelSwitches. So we deliver messages ourselves
     * and leave the checklist controller's connection untouched. Nothing is
     * mutated on NASSP's side by holding this pointer.
     */
    Connector *peer;

    /**
     * A refusal is logged once a session. EnsureConnected() comes here for
     * every command until the LM connects, so a refusing LM would otherwise
     * write several lines per request, at the bridge's polling rate.
     */
    bool logged;

    /**
     * @return true if @p v is a Project Apollo vessel that answered.
     */
    bool ConnectToVessel(VESSEL *v)
    {
        peer = NULL;

        /*
         * GetVesselConnector() folds two very different failures into one
         * NULL: "this is not a Project Apollo vessel" and "the cast produced
         * something that does not validate". The second is the ABI check -
         * ValidationValue sits at a fixed offset inside
         * ProjectApolloConnectorVessel, and that offset is sizeof(VESSEL4)
         * plus padding, so a plugin built against a different Orbiter SDK
         * reads the wrong bytes and fails here with nothing to say why.
         * Doing the two steps separately is the only way to tell them apart
         * from a log file.
         */
        ProjectApolloConnectorVessel *pacv =
            static_cast<ProjectApolloConnectorVessel *>(v);

        char msg[220];
        bool valid = pacv->ValidateVessel();
        sprintf(msg, "LEMLink: ValidateVessel=%s  sizeof(VESSEL4)=%u  "
                     "sizeof(PACV)=%u",
                valid ? "true" : "FALSE",
                (unsigned)sizeof(VESSEL4),
                (unsigned)sizeof(ProjectApolloConnectorVessel));
        const auto log = [this](const char *text) {
            if (!logged)
            {
                oapiWriteLog((char *)text);
            }
        };
        log(msg);

        if (!valid)
        {
            log("LEMLink: layout mismatch - this plugin and LEM.dll disagree "
                "on the class. Check the Orbiter SDK version and vendor/REVISION.");
            return false;
        }

        peer = pacv->GetConnector(VIRTUAL_CONNECTOR_PORT, type);
        if (!peer)
        {
            log("LEMLink: validated, but no MFD_PANEL_INTERFACE registered yet");
            return false;
        }
        return true;
    }

    void Release() { peer = NULL; }

    /**
     * What Connector::SendMessage would do, minus the connection it requires.
     */
    bool Deliver(ConnectorMessage &cm)
    {
        return peer ? peer->ReceiveMessage(this, cm) : false;
    }

    /**
     * @return the item's state, or -1 if there is no such item.
     */
    int GetItemState(const char *name)
    {
        ConnectorMessage cm;
        cm.destination = type;
        cm.messageType = PanelConnector::MFD_PANEL_GET_ITEM_STATE;
        cm.val1.pValue = (void *)name;

        return Deliver(cm) ? cm.val1.iValue : -1;
    }

    /**
     * Moves a control, exactly as a mouse click on the panel would.
     *
     * @p hold matters for spring-loaded switches and is the difference between
     * LUNAR SLEW working and doing nothing at all. PanelSwitches::SetState
     * passes it to SetHeld(), and the switch logic reads
     * `IsSpringLoaded() && !IsHeld()` to decide whether to snap back. Without
     * it a momentary switch returns to centre before the next timestep sees
     * the throw, so the command is silently lost.
     *
     * On a pit the physical switch IS the spring: it returns to centre in the
     * operator's hand, the node reports that, and the bridge commands centre.
     * NASSP's own spring is not just redundant, it actively races us.
     *
     * `guard` stays false. A guarded switch is opened by its own catalogued
     * control here, and LEMBridge's mapping.csv decides that, not this.
     *
     * The return value says little: PanelSwitches::SetState returns true for a
     * name it cannot find, and only writes a debug string (there is a \todo in
     * NASSP about it). So the caller asks GetItemState() about the name first.
     */
    bool SetItemState(const char *name, int value, bool hold)
    {
        ConnectorMessage cm;
        cm.destination  = type;
        cm.messageType  = PanelConnector::MFD_PANEL_SET_ITEM_STATE;
        cm.val1.pValue  = (void *)name;
        cm.val2.iValue  = value;
        cm.val3.bValue  = false;
        cm.val4.bValue  = hold;

        return Deliver(cm) ? cm.val1.bValue : false;
    }
};

/**
 * Our end of the LM display-data connector, if the LM has one.
 *
 * Optional by design. Stock NASSP registers no connector of this type, so
 * `peer` stays NULL and every display read answers "not available" rather than
 * failing to connect at all - the panel half keeps working either way.
 */
class DisplayClient : public Connector
{
public:
    DisplayClient() : peer(NULL) { type = LM_DISPLAY_DATA; }

    Connector *peer;

    bool Attach(ProjectApolloConnectorVessel *pacv)
    {
        peer = pacv->GetConnector(VIRTUAL_CONNECTOR_PORT, type);
        return peer != NULL;
    }

    void Release() { peer = NULL; }
    bool Available() const { return peer != NULL; }

    /**
     * A whole row of eight C&W lights, two bits each. -1 if unavailable, or if
     * it is not one of the five rows: this number comes off the network and
     * NASSP indexes its table with it unchecked.
     */
    int CWLightRow(int row)
    {
        if (!peer || row < 0 || row > 4)
        {
            return -1;
        }
        ConnectorMessage cm;
        cm.destination = type;
        cm.messageType = LMDD_GET_CW_LIGHT_ROW;
        cm.val1.iValue = row;
        return peer->ReceiveMessage(this, cm) ? cm.val1.iValue : -1;
    }

    /** Mission or event timer in seconds. False if unavailable. */
    bool Timer(bool event_timer, double *seconds)
    {
        if (!peer)
        {
            return false;
        }
        ConnectorMessage cm;
        cm.destination = type;
        cm.messageType = event_timer ? LMDD_GET_EVENT_TIMER
                                     : LMDD_GET_MISSION_TIMER;
        if (!peer->ReceiveMessage(this, cm))
        {
            return false;
        }
        *seconds = cm.val1.dValue;
        return true;
    }

    /**
     * One of the messages that fill a caller-owned struct (lmdd_types.h).
     * 1 on success, 0 when there is no display connector at all, -1 when the
     * connector refused the struct - which means this plugin and LEM.dll were
     * built from different lmdd_types.h, and is worth saying rather than
     * reading as "no display".
     */
    template <class T>
    int Query(unsigned msg, T &reply)
    {
        if (!peer)
        {
            return 0;
        }
        reply.size = sizeof(reply);
        ConnectorMessage cm;
        cm.destination = type;
        cm.messageType = msg;
        cm.val1.pValue = &reply;
        cm.val2.bValue = false;
        if (!peer->ReceiveMessage(this, cm))
        {
            return 0;
        }
        return cm.val2.bValue ? 1 : -1;
    }

    /** A meter's displayed value. `ok` says whether the meter exists. */
    double Meter(const char *name, bool *ok)
    {
        *ok = false;
        if (!peer)
        {
            return 0.0;
        }
        ConnectorMessage cm;
        cm.destination = type;
        cm.messageType = LMDD_GET_METER;
        cm.val1.pValue = (void *)name;
        if (!peer->ReceiveMessage(this, cm))
        {
            return 0.0;
        }
        *ok = cm.val2.bValue;
        return cm.val1.dValue;
    }

    /**
     * Sets a continuous knob at full resolution. False when there is no
     * display connector or no continuous knob by that name; `*value` is the
     * knob's value after the call.
     */
    bool SetKnob(const char *name, double *value)
    {
        if (!peer)
        {
            return false;
        }
        ConnectorMessage cm;
        cm.destination = type;
        cm.messageType = LMDD_SET_KNOB;
        cm.val1.pValue = (void *)name;
        cm.val2.bValue = false;
        cm.val3.dValue = *value;
        if (!peer->ReceiveMessage(this, cm) || !cm.val2.bValue)
        {
            return false;
        }
        *value = cm.val1.dValue;
        return true;
    }
};

// ---------------------------------------------------------------------------
//                                  MODULE
// ---------------------------------------------------------------------------

class LEMLinkModule : public oapi::Module
{
public:
    LEMLinkModule(HINSTANCE hDLL)
        : oapi::Module(hDLL), sock(INVALID_SOCKET), note(NULL),
          next_note(0.0), hVessel(NULL),
          wsaUp(false), announced(false)
    {
        strcpy(localip, "127.0.0.1");
    }

    ~LEMLinkModule() { CloseSocket(); }

    void clbkSimulationStart(RenderMode mode);
    void clbkSimulationEnd();
    void clbkPreStep(double simt, double simdt, double mjd);

private:
    SOCKET        sock;
    PanelClient   panel;
    DisplayClient display;      ///< optional; see nassp_patch/patch.md

    /**
     * The gold overlay in the corner of the sim, the way other Orbiter plugins
     * report themselves. Says where to connect and whether an LM was found,
     * which is the difference between "the pit is not wired up" and "the pit
     * is wired to the wrong address" - two failures that otherwise look the
     * same from the panel side.
     */
    NOTEHANDLE note;
    char        localip[64];
    double      next_note;      ///< simt of the next overlay refresh

    /**
     * The vessel we are connected to. Held as an OBJHANDLE rather than a
     * VESSEL* so it can be checked for still existing: the LM stages, and the
     * descent stage becoming a separate vessel would otherwise leave a
     * dangling pointer behind the connector.
     */
    OBJHANDLE hVessel;
    bool      wsaUp;
    bool      announced;

    bool OpenSocket();
    void CloseSocket();
    void UpdateNote(double simt);
    void Service();
    bool EnsureConnected();
    void HandleLine(char *line, SOCKET s, sockaddr_in *from, int fromlen);
};

// ---------------------------------------------------------------------------
//                                  SOCKET
// ---------------------------------------------------------------------------

bool
LEMLinkModule::OpenSocket()
{
    WSADATA wsa;
    if (!wsaUp)
    {
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
        {
            oapiWriteLog((char *)"LEMLink: WSAStartup failed");
            return false;
        }
        wsaUp = true;
    }

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET)
    {
        oapiWriteLog((char *)"LEMLink: socket() failed");
        return false;
    }

    /*
     * Non-blocking, because this is drained from clbkPreStep. A blocking read
     * here would stall the whole simulation whenever the pit is quiet, which
     * is most of the time.
     */
    u_long nb = 1;
    ioctlsocket(sock, FIONBIO, &nb);

    /*
     * This machine only, unless $LEMLINK_ADDRESS names another address to listen
     * on (0.0.0.0 for all of them). Whoever reaches this socket can move every
     * switch in the LM, so being reachable from the LAN is opted into, for a
     * bridge on another machine. Panel nodes only ever talk to the bridge.
     */
    const char *forced = getenv("LEMLINK_ADDRESS");
    snprintf(localip, sizeof(localip), "%s", forced && *forced ? forced : "127.0.0.1");

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(LEMLINK_PORT);

    if (inet_pton(AF_INET, localip, &addr.sin_addr) != 1 ||
        bind(sock, (sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR)
    {
        char msg[128];
        snprintf(msg, sizeof(msg), "LEMLink: bind to %s:%d failed (%d)",
                 localip, LEMLINK_PORT, WSAGetLastError());
        oapiWriteLog(msg);
        closesocket(sock);
        sock = INVALID_SOCKET;
        return false;
    }

    char msg[128];
    snprintf(msg, sizeof(msg), "LEMLink: listening on UDP %s:%d", localip, LEMLINK_PORT);
    oapiWriteLog(msg);
    return true;
}

void
LEMLinkModule::CloseSocket()
{
    if (sock != INVALID_SOCKET)
    {
        closesocket(sock);
        sock = INVALID_SOCKET;
    }
    if (wsaUp)
    {
        WSACleanup();
        wsaUp = false;
    }
}

/**
 * Refreshed about once a second rather than every frame: the text only changes
 * when a vessel appears or goes away, and an annotation rebuild is not free.
 */
void
LEMLinkModule::UpdateNote(double simt)
{
    if (!note || simt < next_note)
    {
        return;
    }
    next_note = simt + 1.0;

    /*
     * Look for the LM here too, not only when a command arrives. The overlay
     * exists to say at a glance whether the pit has something to talk to, and
     * it would otherwise read "no LM" with an LM sitting in front of you until
     * the first switch was thrown - which is the moment you are least likely
     * to be looking at it. Once a second, and it stops as soon as it connects.
     */
    EnsureConnected();

    VESSEL *v = (hVessel && oapiIsVessel(hVessel))
                    ? oapiGetVesselInterface(hVessel) : NULL;

    /*
     * Multi-line, printable ASCII, and SINGLE SPACES ONLY.
     *
     * `\n` is the line separator and is consumed cleanly - this is what NASSP
     * itself does, eight times in one string, to draw a PAD (mcc.cpp, the
     * `oapiAnnotationSetText(NHpad, buffer)` in drawPad()).
     *
     * Two spaces are the trap, and the reason is worth keeping: Orbiter's own
     * ScreenAnnotation::SetText rewrites every "  " in its private copy of the
     * string into "\r\n", and the renderer then splits on "\n" alone. The
     * orphaned \r reaches the font as a character it has no glyph for and
     * draws as a small empty box. So a double space costs a box AND an
     * unasked-for line break. NASSP obeys this silently: not one string
     * literal in mcc.cpp contains two consecutive spaces.
     *
     * None of it is documented and no SDK sample hits it.
     *
     * The font is proportional, so padding cannot align columns either. Use
     * ": " and " | " separators and let the lines be ragged.
     *
     * The vessel name comes from the scenario file, so it is filtered rather
     * than trusted - one stray byte would draw as that same little box.
     */
    char who[32];
    const char *src = v ? v->GetName() : "no LM";
    size_t n = 0;
    for (size_t i = 0; src[i] && n < sizeof(who) - 1; i++)
    {
        unsigned char c = (unsigned char)src[i];
        char out = (c >= 0x20 && c < 0x7F) ? (char)c : '?';

        /*
         * A run of spaces is collapsed, not passed through. Vessel names come
         * out of the scenario file and nothing stops one containing two in a
         * row, which Orbiter would turn into a carriage return and a line
         * break in the middle of the last line of the overlay.
         */
        if (out == ' ' && n > 0 && who[n - 1] == ' ')
        {
            continue;
        }
        who[n++] = out;
    }
    who[n] = '\0';

    char udp[80] = "BIND FAILED";
    if (sock != INVALID_SOCKET)
    {
        sprintf(udp, "%s:%d", localip, LEMLINK_PORT);
    }

    char text[256];
    sprintf(text,
            "LEMLink\n"
            "Panel UDP: %s\n"
            "Telem TCP: %s:%d\n"
            "LEM: %s",
            udp, localip, LEMLINK_TELEM_PORT, who);
    oapiAnnotationSetText(note, text);
}

// ---------------------------------------------------------------------------
//                                 LIFECYCLE
// ---------------------------------------------------------------------------

void
LEMLinkModule::clbkSimulationStart(RenderMode mode)
{
    hVessel   = NULL;
    announced = false;
    next_note = 0.0;
    panel.logged = false;
    OpenSocket();

    // Gold, small, top left, and not exclusive - other modules keep theirs.
    note = oapiCreateAnnotation(false, 0.4, _V(1.0, 0.84, 0.0));

    /*
     * The rect has to fit the text, and both edges matter for different
     * reasons. Too narrow and the renderer word-wraps a long line silently,
     * which reads as mystery line breaks rather than as an overflow. Too
     * short and the lines below the first are simply not what you asked for.
     *
     * Orbiter converts these fractions to pixels once, against a viewport
     * size the annotation cached when it was constructed, so this is set
     * after the render window exists rather than in the constructor.
     *
     * Sizing, from the renderer's own metrics: font height is
     * viewH * size / 35, lines are spaced 1.25 of that, and a proportional
     * Sans averages about 0.55 of it per character. Four lines at size 0.4
     * need about 0.046 of the height; the longest line here is 29 characters,
     * about 0.10 of the width at 16:9. Both are given roughly double that,
     * so a longer vessel name or a different aspect ratio does not wrap.
     */
    oapiAnnotationSetPos(note, 0.02, 0.02, 0.38, 0.14);
    oapiAnnotationSetText(note, (char *)"LEMLink starting up");
}

void
LEMLinkModule::clbkSimulationEnd()
{
    panel.Release();
    display.Release();
    hVessel = NULL;
    if (note)
    {
        oapiDelAnnotation(note);
        note = NULL;
    }
    CloseSocket();
}

/**
 * Finds the LM and connects, or confirms the existing connection is still good.
 *
 * Re-checked every step rather than once at start: the LM does not exist yet
 * in a launch scenario, it is created at staging, and it is replaced by the
 * ascent stage later. Each of those would silently break a connection made
 * once and never revisited.
 */
bool
LEMLinkModule::EnsureConnected()
{
    if (hVessel && oapiIsVessel(hVessel))
    {
        return true;
    }

    hVessel = NULL;
    panel.Release();
    display.Release();

    for (DWORD i = 0; i < oapiGetVesselCount(); i++)
    {
        OBJHANDLE h = oapiGetVesselByIndex(i);
        if (!h)
        {
            continue;
        }
        VESSEL *v = oapiGetVesselInterface(h);
        if (!v)
        {
            continue;
        }

        /*
         * Class names look like "ProjectApollo\LEM". GetVesselConnector()
         * rejects anything that is not a Project Apollo vessel anyway, and
         * validates the cast before using it, so this test only has to pick
         * the LM out of a scenario that also holds a CSM and a launch vehicle.
         */
        const char *cls = v->GetClassName();
        if (!cls || !strstr(cls, "LEM"))
        {
            if (!announced && cls)
            {
                // Every vessel we passed over, so a class name that does not
                // read the way it was assumed to is visible rather than
                // guessed at.
                char msg[200];
                snprintf(msg, sizeof(msg), "LEMLink:   skip %s (%s)", v->GetName(), cls);
                oapiWriteLog(msg);
            }
            continue;
        }

        if (panel.ConnectToVessel(v))
        {
            // Optional extra. Its absence is normal on stock NASSP and must
            // not stop the panel half connecting.
            ProjectApolloConnectorVessel *pacv =
                static_cast<ProjectApolloConnectorVessel *>(v);
            bool disp = display.Attach(pacv);
            oapiWriteLog((char *)(disp
                ? "LEMLink: display data connector present"
                : "LEMLink: no display connector; C&W, timers, meters, "
                  "FDAI, X-pointers, tapes, DSKY, DEDA, AOT and panel lamps unavailable "
                  "(see nassp_patch/patch.md)"));
            hVessel = h;
            char msg[200];
            snprintf(msg, sizeof(msg), "LEMLink: connected to %s (%s)",
                     v->GetName(), cls);
            oapiWriteLog(msg);
            announced = true;
            return true;
        }

        /*
         * Found the LM and it refused. Worth separating from "no LM present":
         * GetVesselConnector() returns NULL either because the vessel is not a
         * Project Apollo one or because ValidateVessel() failed, and the
         * second means our ProjectApolloConnectorVessel layout does not match
         * the one LEM.dll was built with. That is the ABI check, and it is
         * silent unless it is said out loud here.
         */
        if (!panel.logged)
        {
            char msg[200];
            snprintf(msg, sizeof(msg), "LEMLink: %s (%s) is the LM but refused the "
                                       "connector - ABI mismatch or no PanelConnector yet",
                     v->GetName(), cls);
            oapiWriteLog(msg);
            panel.logged = true;
        }
        announced = true;
        return false;
    }

    if (!announced)
    {
        char msg[120];
        sprintf(msg, "LEMLink: no LEM-class vessel among %u",
                (unsigned)oapiGetVesselCount());
        oapiWriteLog(msg);
        announced = true;   // once per session, not once per frame
    }
    return false;
}

// ---------------------------------------------------------------------------
//                                 PROTOCOL
// ---------------------------------------------------------------------------
// One rule for every verb; docs/PROTOCOL.md lists them and what each answers.
//
//   request    <V> [args]              one uppercase letter, then its arguments
//   success    <request> = <fields>
//   failure    <request> ! <code>
//
// Several requests may share a datagram. Each gets exactly one reply line, in a
// datagram of its own. Kept readable on purpose: any of it can be reproduced from
// a terminal while something is broken.
// ---------------------------------------------------------------------------

/// P reports this. It changes whenever a verb or the fields of a reply change.
#define LEMLINK_PROTOCOL 2

/// The longest request. A reply repeats it, and no item name is over 34 characters.
#define MAX_REQUEST 100

/// How many arguments each verb takes, so that no verb has to check for itself.
static const struct { char verb; int fewest, most; } VERBS[] = {
    {'P', 0, 0}, {'S', 2, 3}, {'K', 2, 2}, {'G', 1, 1}, {'N', 1, 1},
    {'W', 1, 1}, {'T', 1, 1}, {'F', 1, 1}, {'X', 1, 1}, {'A', 1, 2},
    {'Y', 0, 0}, {'D', 0, 0}, {'B', 0, 0}, {'O', 0, 0}, {'Q', 0, 0},
    {'I', 0, 0}, {'L', 0, 0}, {'J', 0, 0}, {'C', 0, 0}, {'Z', 0, 0},
};

/** Where one request's answer goes. Whichever is called starts with the request. */
class Answer
{
public:
    Answer(SOCKET s, const sockaddr_in *from, int fromlen, const char *request)
        : s(s), from(from), fromlen(fromlen), request(request) {}

    void Ok(const char *format, ...)
    {
        char out[512];
        int n = snprintf(out, sizeof(out), "%s = ", request);
        va_list fields;
        va_start(fields, format);
        vsnprintf(out + n, sizeof(out) - n - 1, format, fields);
        va_end(fields);
        Send(strcat(out, "\n"));
    }

    void Fail(const char *code)
    {
        char out[MAX_REQUEST + 40];
        snprintf(out, sizeof(out), "%s ! %s\n", request, code);
        Send(out);
    }

private:
    SOCKET s;
    const sockaddr_in *from;
    int fromlen;
    const char *request;

    void Send(const char *text)
    {
        sendto(s, text, (int)strlen(text), 0, (const sockaddr *)from, fromlen);
    }
};

/** A whole number and nothing else. atoi reads "abc" as 0, and that moved a switch. */
static bool
WholeNumber(const char *text, long *value, int base = 10)
{
    char *end;
    *value = strtol(text, &end, base);
    return end != text && !*end;
}

/** A selector argument: a whole number from 0 to @p highest. */
static bool
Selector(const char *text, long *value, long highest)
{
    return WholeNumber(text, value) && *value >= 0 && *value <= highest;
}

/** A plain decimal number. strtod alone also takes nan, inf and hex, which no knob means. */
static bool
Decimal(const char *text, double *value)
{
    char *end;
    *value = strtod(text, &end);
    return text[strspn(text, "-.0123456789")] == '\0' && end != text && !*end;
}

/**
 * A string as one wire field. Replies are split on spaces, and a DSKY or DEDA blank
 * digit IS a space, so blanks travel as '_' and an empty string as '-'. So does
 * anything unprintable: a vessel's name comes out of the .scn file and can be any
 * bytes. The field count of a reply therefore never changes with what it carries.
 */
static void
WireText(char *dst, size_t n, const char *src)
{
    size_t i = 0;
    for (; src[i] && i + 1 < n; i++)
    {
        unsigned char ch = (unsigned char)src[i];
        dst[i] = (ch > 0x20 && ch < 0x7F) ? (char)ch : '_';
    }
    if (i == 0 && n > 1)
    {
        dst[i++] = '-';
    }
    dst[i] = '\0';
}

/** One of the display reads that fill a struct. False once it has answered the failure. */
template <class T>
static bool
ReadDisplay(DisplayClient &display, unsigned msg, T &r, Answer &reply)
{
    int got = display.Query(msg, r);
    if (got <= 0)
    {
        reply.Fail(got < 0 ? "display-mismatch" : "no-display");
    }
    return got > 0;
}

void
LEMLinkModule::HandleLine(char *line, SOCKET s, sockaddr_in *from, int fromlen)
{
    // Both ends trimmed: the reply repeats this, and the bridge looks it up by it.
    line += strspn(line, " \t");
    size_t len = strlen(line);
    while (len && (line[len - 1] == ' ' || line[len - 1] == '\t'))
    {
        line[--len] = '\0';
    }
    if (!len)
    {
        return;
    }

    char request[MAX_REQUEST + 1];
    snprintf(request, sizeof(request), "%s", line);
    Answer reply(s, from, fromlen, request);
    if (len > MAX_REQUEST)
    {
        return reply.Fail("bad-args");
    }

    const char *word = strtok(line, " \t");
    char *arg[3];
    int nargs = 0;
    for (char *t; (t = strtok(NULL, " \t")) != NULL; nargs++)
    {
        if (nargs < 3)
        {
            arg[nargs] = t;
        }
    }

    const char verb = word[1] ? '\0' : word[0];
    int fewest = -1, most = -1;
    for (const auto &known : VERBS)
    {
        if (known.verb == verb)
        {
            fewest = known.fewest;
            most = known.most;
        }
    }
    if (fewest < 0)
    {
        return reply.Fail("bad-verb");
    }
    if (nargs < fewest || nargs > most)
    {
        return reply.Fail("bad-args");
    }
    if (verb != 'P' && !EnsureConnected())
    {
        return reply.Fail("no-lm");
    }

    long which;
    switch (verb)
    {
    case 'P':
    {
        EnsureConnected();      // a status verb should answer with the truth
        VESSEL *v = (hVessel && oapiIsVessel(hVessel))
                        ? oapiGetVesselInterface(hVessel) : NULL;
        LMDDLamps probe = {};
        int got = v ? display.Query(LMDD_GET_PANEL_LAMPS, probe) : 0;
        char name[64];
        WireText(name, sizeof(name), v ? v->GetName() : "");
        return reply.Ok("%d %s %.3f %s", LEMLINK_PROTOCOL,
                        !v ? "-" : got > 0 ? "patched" : got < 0 ? "mismatch" : "stock",
                        oapiGetSimTime(), name);
    }

    case 'S':
    {
        // NASSP stores whatever it is given, range unchecked, and answers -1 for a
        // name it cannot find. So a control set to -1 could never be set again.
        long value, hold = 0;
        if (!WholeNumber(arg[1], &value) || value < 0 ||
            (nargs == 3 && !Selector(arg[2], &hold, 1)))
        {
            return reply.Fail("bad-args");
        }
        if (panel.GetItemState(arg[0]) < 0 ||
            !panel.SetItemState(arg[0], (int)value, hold != 0))
        {
            return reply.Fail("no-item");
        }
        // Read back in the same frame, so a spring or a clamp shows here.
        return reply.Ok("%d", panel.GetItemState(arg[0]));
    }

    case 'K':
    {
        double value;
        if (!Decimal(arg[1], &value))
        {
            return reply.Fail("bad-args");
        }
        if (panel.GetItemState(arg[0]) < 0)
        {
            return reply.Fail("no-item");
        }
        if (display.Available())
        {
            if (!display.SetKnob(arg[0], &value))
            {
                return reply.Fail("bad-args");      // it exists, but is not a knob
            }
        }
        else
        {
            // A stock LEM.dll has no full resolution: whole numbers, by the panel.
            panel.SetItemState(arg[0], value > 0 ? (int)floor(value + 0.5) : 0, false);
            value = panel.GetItemState(arg[0]);
        }
        return reply.Ok("%.4f", value);
    }

    case 'G':
    {
        int state = panel.GetItemState(arg[0]);
        if (state < 0)
        {
            return reply.Fail("no-item");
        }
        return reply.Ok("%d", state);
    }

    case 'N':
    {
        if (!display.Available())
        {
            return reply.Fail("no-display");
        }
        bool ok = false;
        double value = display.Meter(arg[0], &ok);
        if (!ok)
        {
            return reply.Fail("no-item");
        }
        return reply.Ok("%.4f", value);
    }

    case 'W':
    {
        if (!Selector(arg[0], &which, 4))
        {
            return reply.Fail("bad-args");
        }
        int packed = display.CWLightRow((int)which);
        if (packed < 0)
        {
            return reply.Fail("no-display");
        }
        return reply.Ok("%x", packed);
    }

    case 'T':
    {
        double seconds;
        if (!Selector(arg[0], &which, 1))
        {
            return reply.Fail("bad-args");
        }
        if (!display.Timer(which != 0, &seconds))
        {
            return reply.Fail("no-display");
        }
        return reply.Ok("%.3f", seconds);
    }

    case 'F':
    {
        if (!Selector(arg[0], &which, 1))
        {
            return reply.Fail("bad-args");
        }
        LMDDFdai r = {};
        r.which = (int)which;
        if (ReadDisplay(display, LMDD_GET_FDAI, r, reply))
        {
            reply.Ok("%d %.4f %.4f %.4f %.4f %.4f %.4f %.3f %.3f %.3f",
                     r.powered, r.rate[0], r.rate[1], r.rate[2],
                     r.error[0], r.error[1], r.error[2],
                     r.ball[0], r.ball[1], r.ball[2]);
        }
        return;
    }

    case 'X':
    {
        if (!Selector(arg[0], &which, 1))
        {
            return reply.Fail("bad-args");
        }
        LMDDXPointer r = {};
        r.which = (int)which;
        if (ReadDisplay(display, LMDD_GET_XPOINTER, r, reply))
        {
            reply.Ok("%d %d %.4f %.4f %.4f %.4f", r.powered, r.fine,
                     r.needle[0], r.needle[1], r.raw[0], r.raw[1]);
        }
        return;
    }

    case 'A':
    {
        long address, count = 1;
        if (!WholeNumber(arg[0], &address, 8) || address < 0 || address > 03777 ||
            (nargs == 2 && (!WholeNumber(arg[1], &count) || count < 1 || count > 16)))
        {
            return reply.Fail("bad-args");
        }
        LMDDErasable r = {};
        r.address = (int)address;
        r.count = (int)count;
        if (ReadDisplay(display, LMDD_GET_ERASABLE, r, reply))
        {
            char words[16 * 7 + 1];
            int n = 0;
            for (int i = 0; i < count; i++)
            {
                n += sprintf(words + n, i ? " %o" : "%o", r.words[i]);
            }
            reply.Ok("%s", words);
        }
        return;
    }

    case 'Y':
    {
        LMDDTape r = {};
        if (ReadDisplay(display, LMDD_GET_TAPE, r, reply))
        {
            reply.Ok("%d %d %d %d %d %.2f %.2f %.3f %.4f", r.powered,
                     r.monitor_on, r.power_fail, r.signal_fail, r.timing_fail,
                     r.range_pos, r.rate_pos, r.range_m, r.rate_mps);
        }
        return;
    }

    case 'D':
    {
        LMDDDsky r = {};
        if (ReadDisplay(display, LMDD_GET_DSKY, r, reply))
        {
            char pg[8], vb[8], nn[8], a1[12], a2[12], a3[12];
            WireText(pg, sizeof(pg), r.prog);
            WireText(vb, sizeof(vb), r.verb);
            WireText(nn, sizeof(nn), r.noun);
            WireText(a1, sizeof(a1), r.r1);
            WireText(a2, sizeof(a2), r.r2);
            WireText(a3, sizeof(a3), r.r3);
            reply.Ok("%d %d %x %s %s %s %s %s %s", r.segment_power,
                     r.status_power, r.lamps, pg, vb, nn, a1, a2, a3);
        }
        return;
    }

    case 'B':
    {
        LMDDDeda r = {};
        if (ReadDisplay(display, LMDD_GET_DEDA, r, reply))
        {
            char ad[8], da[12];
            WireText(ad, sizeof(ad), r.adr);
            WireText(da, sizeof(da), r.data);
            reply.Ok("%d %d %s %s", r.powered, r.opr_err, ad, da);
        }
        return;
    }

    case 'O':
    {
        LMDDAot r = {};
        if (ReadDisplay(display, LMDD_GET_AOT, r, reply))
        {
            reply.Ok("%d %d %.3f %.6f", r.shaft, r.dimmer,
                     r.counter_deg, r.reticle_rad);
        }
        return;
    }

    case 'Q':
    {
        LMDDLamps r = {};
        if (ReadDisplay(display, LMDD_GET_PANEL_LAMPS, r, reply))
        {
            reply.Ok("%x", r.lit);
        }
        return;
    }

    case 'I':
    {
        LMDDAttitude r = {};
        if (ReadDisplay(display, LMDD_GET_ATTITUDE, r, reply))
        {
            reply.Ok("%d %d %d %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
                     r.imu_powered, r.imu_caged, r.gasta_powered,
                     r.imu[0], r.imu[1], r.imu[2], r.gasta[0], r.gasta[1], r.gasta[2],
                     r.ags[0], r.ags[1], r.ags[2], r.ordeal_pitch[0], r.ordeal_pitch[1]);
        }
        return;
    }

    case 'L':
    {
        LMDDLegends r = {};
        if (ReadDisplay(display, LMDD_GET_VC_LEGENDS, r, reply))
        {
            reply.Ok("%x %.3f", r.lit, r.numeric);
        }
        return;
    }

    case 'J':
    {
        LMDDLighting r = {};
        if (ReadDisplay(display, LMDD_GET_LIGHTING, r, reply))
        {
            reply.Ok("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %d %d",
                     r.integral, r.numeric, r.anun, r.dsky, r.flood_cdr,
                     r.flood_lmp, r.flood_side, r.ordeal, r.side_8_11,
                     r.side_12_14_16);
        }
        return;
    }

    case 'C':
    {
        // Needs no patch. NASSP's own target is LEM::agc.GetCSM(), which this cannot
        // see; with one CSM in a scenario they are the same vessel.
        VESSEL *v = oapiGetVesselInterface(hVessel);
        VECTOR3 lp, lv;
        v->GetGlobalPos(lp);
        v->GetGlobalVel(lv);
        VESSEL *best = NULL;
        double bestd = 0;
        for (DWORD i = 0; i < oapiGetVesselCount(); i++)
        {
            OBJHANDLE h = oapiGetVesselByIndex(i);
            if (h == hVessel)
                continue;
            VESSEL *o = oapiGetVesselInterface(h);
            const char *cls = o->GetClassName();
            if (!cls || !strstr(cls, "ProjectApollo") || !strstr(cls, "Saturn"))
                continue;
            VECTOR3 op;
            o->GetGlobalPos(op);
            double d = length(op - lp);
            if (!best || d < bestd)
            {
                best = o;
                bestd = d;
            }
        }
        if (!best)
        {
            return reply.Fail("no-csm");
        }
        VECTOR3 op, ov;
        best->GetGlobalPos(op);
        best->GetGlobalVel(ov);
        op = op - lp;
        ov = ov - lv;
        char name[64];
        WireText(name, sizeof(name), best->GetName());
        return reply.Ok("%s %.2f %.2f %.2f %.4f %.4f %.4f",
                        name, op.x, op.y, op.z, ov.x, ov.y, ov.z);
    }

    case 'Z':
    {
        // From Orbiter itself, not NASSP, so a recorded instrument can be judged
        // against the actual vehicle. Needs no patch.
        VESSEL *v = oapiGetVesselInterface(hVessel);
        VECTOR3 hz = _V(0, 0, 0), gs = _V(0, 0, 0), w = _V(0, 0, 0),
                th = _V(0, 0, 0), pos = _V(0, 0, 0);
        MATRIX3 rot;
        v->GetGroundspeedVector(FRAME_HORIZON, hz);
        v->GetGroundspeedVector(FRAME_LOCAL, gs);
        v->GetAngularVel(w);
        v->GetThrustVector(th);
        // The vessel-to-global rotation the IMU itself reads (IMU::Timestep), and
        // the position from the gravity reference, so a recording can recover the
        // platform's orientation in space.
        v->GetRotationMatrix(rot);
        OBJHANDLE ref = v->GetGravityRef();
        if (ref)
            v->GetRelativePos(ref, pos);
        return reply.Ok("%.3f %.2f %.2f %.3f %.3f %.3f %.3f %.5f %.5f %.5f "
                        "%.5f %.5f %.5f %.1f %.1f "
                        "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f "
                        "%.1f %.1f %.1f",
                        oapiGetSimTime(), v->GetAltitude(ALTMODE_GROUND),
                        v->GetAltitude(ALTMODE_MEANRAD), hz.y, gs.x, gs.y, gs.z,
                        w.x, w.y, w.z, v->GetPitch(), v->GetBank(), v->GetYaw(),
                        length(th), v->GetMass(),
                        rot.m11, rot.m12, rot.m13, rot.m21, rot.m22, rot.m23,
                        rot.m31, rot.m32, rot.m33, pos.x, pos.y, pos.z);
    }

    default:
        return reply.Fail("bad-verb");      // in VERBS, with no case here
    }
}

void
LEMLinkModule::Service()
{
    if (sock == INVALID_SOCKET)
    {
        return;
    }

    for (int n = 0; n < MAX_PACKETS_PER_STEP; n++)
    {
        char        buf[RECV_BUF];
        sockaddr_in from;
        int         fromlen = sizeof(from);

        int len = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                           (sockaddr *)&from, &fromlen);
        if (len <= 0)
        {
            return;     // WSAEWOULDBLOCK on an empty queue: nothing to do
        }
        buf[len] = '\0';

        // Split on newlines in place; a batched dump arrives as one datagram.
        char *p = buf;
        while (*p)
        {
            char *nl = strpbrk(p, "\r\n");
            if (nl)
            {
                *nl = '\0';
            }
            HandleLine(p, sock, &from, fromlen);
            if (!nl)
            {
                break;
            }
            p = nl + 1;
        }
    }
}

void
LEMLinkModule::clbkPreStep(double simt, double simdt, double mjd)
{
    Service();
    UpdateNote(simt);
}

// ---------------------------------------------------------------------------
//                                 DLL ENTRY
// ---------------------------------------------------------------------------

DLLCLBK void
InitModule(HINSTANCE hDLL)
{
    oapiRegisterModule(new LEMLinkModule(hDLL));
}

DLLCLBK void
ExitModule(HINSTANCE hDLL)
{
    // Orbiter deletes the registered module itself.
}

/*** end of file ***/
