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
#include <iphlpapi.h>
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
 * this type, GetConnector() returns NULL, and the display verbs answer "E".
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
    PanelClient() : peer(NULL) { type = MFD_PANEL_INTERFACE; }

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
        oapiWriteLog(msg);

        if (!valid)
        {
            oapiWriteLog((char *)"LEMLink: layout mismatch - this plugin "
                                 "and LEM.dll disagree on the class. Check "
                                 "the Orbiter SDK version and vendor/REVISION.");
            return false;
        }

        peer = pacv->GetConnector(VIRTUAL_CONNECTOR_PORT, type);
        if (!peer)
        {
            oapiWriteLog((char *)"LEMLink: validated, but no "
                                 "MFD_PANEL_INTERFACE registered yet");
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
     * Note the return value is close to worthless: PanelSwitches::SetState
     * returns true for a name it cannot find, and only writes a debug string
     * (there is a \todo in NASSP about it). Read the item back if it matters.
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

    /** A whole row of eight C&W lights, two bits each. -1 if unavailable. */
    int CWLightRow(int row)
    {
        if (!peer)
        {
            return -1;
        }
        ConnectorMessage cm;
        cm.destination = type;
        cm.messageType = LMDD_GET_CW_LIGHT_ROW;
        cm.val1.iValue = row;
        return peer->ReceiveMessage(this, cm) ? cm.val1.iValue : -1;
    }

    /** Mission or event timer in seconds, or -1 if unavailable. */
    double Timer(bool event_timer)
    {
        if (!peer)
        {
            return -1.0;
        }
        ConnectorMessage cm;
        cm.destination = type;
        cm.messageType = event_timer ? LMDD_GET_EVENT_TIMER
                                     : LMDD_GET_MISSION_TIMER;
        return peer->ReceiveMessage(this, cm) ? cm.val1.dValue : -1.0;
    }

    /**
     * One of the messages that fill a caller-owned struct (lmdd_types.h).
     * The struct's `size` must already be set. 1 on success, 0 when there is
     * no display connector at all, -1 when the connector refused the struct -
     * which means this plugin and LEM.dll were built from different
     * lmdd_types.h, and is worth saying rather than reading as "no display".
     */
    int Query(unsigned msg, void *reply)
    {
        if (!peer)
        {
            return 0;
        }
        ConnectorMessage cm;
        cm.destination = type;
        cm.messageType = msg;
        cm.val1.pValue = reply;
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
     * A continuous knob at full resolution: set it when `set`, then read it.
     * False when there is no display connector or no continuous knob by that
     * name; `*value` is the knob's value after the call.
     */
    bool Knob(const char *name, bool set, double *value)
    {
        if (!peer)
        {
            return false;
        }
        ConnectorMessage cm;
        cm.destination = type;
        cm.messageType = set ? LMDD_SET_KNOB : LMDD_GET_KNOB;
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
        strcpy(localip, "0.0.0.0");
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
    void FindLocalIP();
    void UpdateNote(double simt);
    void Service();
    bool EnsureConnected();
    void HandleLine(char *line, SOCKET s, sockaddr_in *from, int fromlen);
    void Reply(SOCKET s, sockaddr_in *from, int fromlen, const char *text);
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

    // 0.0.0.0 so a node or bridge on the LAN reaches us, not only localhost.
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(LEMLINK_PORT);

    if (bind(sock, (sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR)
    {
        char msg[128];
        sprintf(msg, "LEMLink: bind to %d failed (%d)",
                LEMLINK_PORT, WSAGetLastError());
        oapiWriteLog(msg);
        closesocket(sock);
        sock = INVALID_SOCKET;
        return false;
    }

    char msg[128];
    sprintf(msg, "LEMLink: listening on UDP %d", LEMLINK_PORT);
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
 * The address this Orbiter machine shows for itself: its primary interface.
 *
 * The socket binds 0.0.0.0, which is right but useless to read off a screen,
 * and "whatever reaches the internet" changes the moment a phone tether or a
 * VPN takes the default route. So the label is deterministic instead: the
 * IPv4 address on the lowest-numbered interface that is up, skipping loopback
 * and link-local. Built-in NICs are numbered at boot; anything added later (a
 * USB tether, a VM bridge) gets a higher number. $LEMLINK_ADDRESS overrides it.
 */
void
LEMLinkModule::FindLocalIP()
{
    strcpy(localip, "0.0.0.0");

    const char *forced = getenv("LEMLINK_ADDRESS");
    if (forced && *forced)
    {
        strncpy(localip, forced, sizeof(localip) - 1);
        localip[sizeof(localip) - 1] = '\0';
        return;
    }

    ULONG size = 16 * 1024;
    IP_ADAPTER_ADDRESSES *list = NULL;
    ULONG rc = ERROR_BUFFER_OVERFLOW;
    for (int tries = 0; tries < 3 && rc == ERROR_BUFFER_OVERFLOW; tries++)
    {
        free(list);
        list = (IP_ADAPTER_ADDRESSES *)malloc(size);
        if (!list)
        {
            return;
        }
        rc = GetAdaptersAddresses(AF_INET,
                                  GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
                                  GAA_FLAG_SKIP_DNS_SERVER, NULL, list, &size);
    }
    if (rc == NO_ERROR)
    {
        DWORD best = 0xFFFFFFFF;
        for (IP_ADAPTER_ADDRESSES *a = list; a; a = a->Next)
        {
            if (a->OperStatus != IfOperStatusUp ||
                a->IfType == IF_TYPE_SOFTWARE_LOOPBACK || a->IfIndex >= best)
            {
                continue;
            }
            for (IP_ADAPTER_UNICAST_ADDRESS *u = a->FirstUnicastAddress; u; u = u->Next)
            {
                sockaddr_in *in = (sockaddr_in *)u->Address.lpSockaddr;
                unsigned long host = ntohl(in->sin_addr.s_addr);
                if ((host >> 24) == 127 || (host >> 16) == 0xA9FE)   // loopback, 169.254
                {
                    continue;
                }
                const char *txt = inet_ntoa(in->sin_addr);
                if (txt)
                {
                    strncpy(localip, txt, sizeof(localip) - 1);
                    localip[sizeof(localip) - 1] = '\0';
                    best = a->IfIndex;
                }
                break;
            }
        }
    }
    free(list);
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

    char text[256];
    sprintf(text,
            "LEMLink\n"
            "Panel UDP: %s:%d\n"
            "Telem TCP: %s:%d\n"
            "LEM: %s",
            localip, LEMLINK_PORT, localip, LEMLINK_TELEM_PORT, who);
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
    OpenSocket();
    FindLocalIP();

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
                sprintf(msg, "LEMLink:   skip %s (%s)", v->GetName(), cls);
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
            sprintf(msg, "LEMLink: connected to %s (%s)",
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
        char msg[200];
        sprintf(msg, "LEMLink: %s (%s) is the LM but refused the "
                     "connector - ABI mismatch or no PanelConnector yet",
                v->GetName(), cls);
        oapiWriteLog(msg);
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
// Line based, one command per line, several lines to a datagram allowed. Kept
// readable on purpose - the same reason the node protocol is readable, which
// is that you can reproduce any of it from a terminal while something is
// broken.
//
//   S <item> <value>   set a control, answered "K <item> <value>" or "E <item>"
//   G <item>           read a control, answered "V <item> <value>"
//   P                  ping, answered "A <vessel>" or "A -" when unconnected
// ---------------------------------------------------------------------------

void
LEMLinkModule::Reply(SOCKET s, sockaddr_in *from, int fromlen, const char *text)
{
    sendto(s, text, (int)strlen(text), 0, (sockaddr *)from, fromlen);
}

/*
 * A display string as one wire field. Replies are split on spaces, and a DSKY
 * or DEDA blank digit IS a space, so blanks travel as '_' and an empty string
 * as '-'. The field count of a reply therefore never changes with what the
 * display happens to show.
 */
static void
WireText(char *dst, size_t n, const char *src)
{
    size_t i = 0;
    for (; src[i] && i + 1 < n; i++)
    {
        dst[i] = (src[i] == ' ') ? '_' : src[i];
    }
    if (i == 0 && n > 1)
    {
        dst[i++] = '-';
    }
    dst[i] = '\0';
}

void
LEMLinkModule::HandleLine(char *line, SOCKET s, sockaddr_in *from, int fromlen)
{
    char out[512];      // the Z reply is about 270 bytes since it carries the rotation

    while (*line == ' ' || *line == '\t')
    {
        line++;
    }
    if (!*line)
    {
        return;
    }

    char verb = *line++;
    while (*line == ' ')
    {
        line++;
    }

    if (verb == 'P')
    {
        EnsureConnected();      // a status verb should answer with the truth
        VESSEL *v = (hVessel && oapiIsVessel(hVessel))
                        ? oapiGetVesselInterface(hVessel) : NULL;
        sprintf(out, "A %s\n", v ? v->GetName() : "-");
        Reply(s, from, fromlen, out);
        return;
    }

    if (!EnsureConnected())
    {
        Reply(s, from, fromlen, "E no-lm\n");
        return;
    }

    // "<item> <value>" - the item name never contains a space.
    char *sp = strchr(line, ' ');
    if (sp)
    {
        *sp = '\0';
    }

    // "W <row>"  -> "R <row> <packed>", eight C&W lights, two bits each
    // "T <0|1>"   -> "M <which> <seconds>", mission or event timer
    // "N <meter>" -> "U <meter> <value>"
    if (verb == 'W' || verb == 'T' || verb == 'N')
    {
        if (!EnsureConnected() || !display.Available())
        {
            Reply(s, from, fromlen, "E no-display\n");
            return;
        }
        if (verb == 'W')
        {
            int row = atoi(line);
            sprintf(out, "R %d %d\n", row, display.CWLightRow(row));
        }
        else if (verb == 'T')
        {
            int which = atoi(line);
            sprintf(out, "M %d %.3f\n", which, display.Timer(which != 0));
        }
        else
        {
            char *sp2 = strchr(line, ' ');
            if (sp2) { *sp2 = '\0'; }
            bool ok = false;
            double v = display.Meter(line, &ok);
            if (ok) { sprintf(out, "U %.80s %.4f\n", line, v); }
            else    { sprintf(out, "E %.80s no-meter\n", line); }
        }
        Reply(s, from, fromlen, out);
        return;
    }

    // Everything else the panel draws, through the patched connector. Each
    // answers with the lowercase of its own letter, and all of them answer
    // "E no-display" on stock NASSP, like W/T/N:
    //
    //   "F <0|1>" -> "f <which> <powered> <rate r p y> <error r p y> <ball r p y>"
    //   "X <0|1>" -> "x <which> <powered> <fine> <needle fwd lat> <raw fwd lat>"
    //   "Y"       -> "y <powered> <monitor> <pfail> <sfail> <tfail>
    //                   <range_pos> <rate_pos> <range_m> <rate_mps>"
    //   "D"       -> "d <segpwr> <statpwr> <lamps-hex> <prog> <verb> <noun> <r1> <r2> <r3>"
    //   "B"       -> "b <powered> <opr_err> <adr> <data>"
    //   "O"       -> "o <shaft> <dimmer> <counter_deg> <reticle_rad>"
    //   "Q"       -> "q <lit-hex>", LMDD_LAMP_* bits: contact, component
    //                caution, stage-seq and power-failure lamps
    //   "I"       -> "i <imu_pwr> <caged> <gasta_pwr> <imu x y z>
    //                   <gasta r p y> <ags r p y> <ordeal1> <ordeal2>", degrees
    //   "L"       -> "l <lit-hex> <numeric>", LMDD_LEGEND_* bits: the legends
    //                only the VC lights (X-pointer source and scale, tape
    //                mode, RCS He x10), and the NUM dimmer level they draw at
    //   "J"       -> "j <integral> <numeric> <anun> <dsky> <flood_cdr>
    //                   <flood_lmp> <flood_side> <ordeal> <side_8_11>
    //                   <side_12_14_16>": every lighting channel, 0..1 as the
    //                VC scales it, and the two side-panel integral switches
    // "A <octal address> [count]" -> "a <address> <word> ...", octal: LGC
    // erasable memory, read-only (patch only). See LMDDErasable.
    if (verb == 'A')
    {
        LMDDErasable r;
        memset(&r, 0, sizeof(r));
        r.size = sizeof(r);
        // The dispatcher has already cut the line at its first space (`sp`),
        // so the address is in `line` and the count, if any, follows `sp`.
        r.address = (int)strtol(line, NULL, 8);
        r.count = sp ? atoi(sp + 1) : 1;
        int got = display.Query(LMDD_GET_ERASABLE, &r);
        if (got > 0)
        {
            int n = sprintf(out, "a %o", r.address);
            for (int i = 0; i < r.count && i < 16; i++)
                n += sprintf(out + n, " %o", r.words[i]);
            sprintf(out + n, "\n");
            Reply(s, from, fromlen, out);
        }
        else
        {
            Reply(s, from, fromlen, got < 0 ? "E display-mismatch\n" : "E no-display\n");
        }
        return;
    }

    if (verb == 'F' || verb == 'X' || verb == 'Y' ||
        verb == 'D' || verb == 'B' || verb == 'O' || verb == 'Q' ||
        verb == 'I' || verb == 'L' || verb == 'J')
    {
        int which = atoi(line) != 0 ? 1 : 0;
        int got = 0;
        if (verb == 'F')
        {
            LMDDFdai r;
            memset(&r, 0, sizeof(r));
            r.size = sizeof(r);
            r.which = which;
            got = display.Query(LMDD_GET_FDAI, &r);
            sprintf(out, "f %d %d %.4f %.4f %.4f %.4f %.4f %.4f %.3f %.3f %.3f\n",
                    which, r.powered, r.rate[0], r.rate[1], r.rate[2],
                    r.error[0], r.error[1], r.error[2],
                    r.ball[0], r.ball[1], r.ball[2]);
        }
        else if (verb == 'X')
        {
            LMDDXPointer r;
            memset(&r, 0, sizeof(r));
            r.size = sizeof(r);
            r.which = which;
            got = display.Query(LMDD_GET_XPOINTER, &r);
            sprintf(out, "x %d %d %d %.4f %.4f %.4f %.4f\n", which, r.powered,
                    r.fine, r.needle[0], r.needle[1], r.raw[0], r.raw[1]);
        }
        else if (verb == 'Y')
        {
            LMDDTape r;
            memset(&r, 0, sizeof(r));
            r.size = sizeof(r);
            got = display.Query(LMDD_GET_TAPE, &r);
            sprintf(out, "y %d %d %d %d %d %.2f %.2f %.3f %.4f\n", r.powered,
                    r.monitor_on, r.power_fail, r.signal_fail, r.timing_fail,
                    r.range_pos, r.rate_pos, r.range_m, r.rate_mps);
        }
        else if (verb == 'D')
        {
            LMDDDsky r;
            memset(&r, 0, sizeof(r));
            r.size = sizeof(r);
            got = display.Query(LMDD_GET_DSKY, &r);
            char pg[8], vb[8], nn[8], a1[12], a2[12], a3[12];
            WireText(pg, sizeof(pg), r.prog);
            WireText(vb, sizeof(vb), r.verb);
            WireText(nn, sizeof(nn), r.noun);
            WireText(a1, sizeof(a1), r.r1);
            WireText(a2, sizeof(a2), r.r2);
            WireText(a3, sizeof(a3), r.r3);
            sprintf(out, "d %d %d %x %s %s %s %s %s %s\n", r.segment_power,
                    r.status_power, r.lamps, pg, vb, nn, a1, a2, a3);
        }
        else if (verb == 'B')
        {
            LMDDDeda r;
            memset(&r, 0, sizeof(r));
            r.size = sizeof(r);
            got = display.Query(LMDD_GET_DEDA, &r);
            char ad[8], da[12];
            WireText(ad, sizeof(ad), r.adr);
            WireText(da, sizeof(da), r.data);
            sprintf(out, "b %d %d %s %s\n", r.powered, r.opr_err, ad, da);
        }
        else if (verb == 'Q')
        {
            LMDDLamps r;
            memset(&r, 0, sizeof(r));
            r.size = sizeof(r);
            got = display.Query(LMDD_GET_PANEL_LAMPS, &r);
            sprintf(out, "q %x\n", r.lit);
        }
        else if (verb == 'L')
        {
            LMDDLegends r;
            memset(&r, 0, sizeof(r));
            r.size = sizeof(r);
            got = display.Query(LMDD_GET_VC_LEGENDS, &r);
            sprintf(out, "l %x %.3f\n", r.lit, r.numeric);
        }
        else if (verb == 'J')
        {
            LMDDLighting r;
            memset(&r, 0, sizeof(r));
            r.size = sizeof(r);
            got = display.Query(LMDD_GET_LIGHTING, &r);
            sprintf(out, "j %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %d %d\n",
                    r.integral, r.numeric, r.anun, r.dsky, r.flood_cdr,
                    r.flood_lmp, r.flood_side, r.ordeal, r.side_8_11,
                    r.side_12_14_16);
        }
        else if (verb == 'I')
        {
            LMDDAttitude r;
            memset(&r, 0, sizeof(r));
            r.size = sizeof(r);
            got = display.Query(LMDD_GET_ATTITUDE, &r);
            sprintf(out, "i %d %d %d %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
                    r.imu_powered, r.imu_caged, r.gasta_powered,
                    r.imu[0], r.imu[1], r.imu[2], r.gasta[0], r.gasta[1], r.gasta[2],
                    r.ags[0], r.ags[1], r.ags[2], r.ordeal_pitch[0], r.ordeal_pitch[1]);
        }
        else
        {
            LMDDAot r;
            memset(&r, 0, sizeof(r));
            r.size = sizeof(r);
            got = display.Query(LMDD_GET_AOT, &r);
            sprintf(out, "o %d %d %.3f %.6f\n", r.shaft, r.dimmer,
                    r.counter_deg, r.reticle_rad);
        }
        if (got == 0)
        {
            Reply(s, from, fromlen, "E no-display\n");
        }
        else if (got < 0)
        {
            Reply(s, from, fromlen, "E display-mismatch\n");
        }
        else
        {
            Reply(s, from, fromlen, out);
        }
        return;
    }

    // "C" -> ground truth for the rendezvous radar: the nearest Apollo CSM
    // (a vessel whose class is ProjectApollo's Saturn), relative to the LM,
    // in Orbiter's global frame. lm_rr.cpp measures exactly these: range is
    // |pos| and range rate is vel along pos, while the radar holds lock.
    //
    //   "c <name> <pos x y z> <vel x y z>"    metres and m/s, CSM minus LM
    //   "c - 0 0 0 0 0 0"                     no CSM in the scenario
    //
    // Needs no patch. NASSP's own target is LEM::agc.GetCSM(), which this
    // cannot see; with one CSM in a scenario they are the same vessel.
    if (verb == 'C')
    {
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
            Reply(s, from, fromlen, "c - 0 0 0 0 0 0\n");
            return;
        }
        VECTOR3 op, ov;
        best->GetGlobalPos(op);
        best->GetGlobalVel(ov);
        op = op - lp;
        ov = ov - lv;
        // One whitespace-free printable token, so the reply splits on spaces.
        char name[64];
        const char *src = best->GetName();
        size_t n = 0;
        for (size_t i = 0; src && src[i] && n < sizeof(name) - 1; i++)
        {
            unsigned char ch = (unsigned char)src[i];
            name[n++] = (ch > 0x20 && ch < 0x7F) ? (char)ch : '_';
        }
        name[n] = '\0';
        if (n == 0)
            strcpy(name, "_");
        sprintf(out, "c %s %.2f %.2f %.2f %.4f %.4f %.4f\n",
                name, op.x, op.y, op.z, ov.x, ov.y, ov.z);
        Reply(s, from, fromlen, out);
        return;
    }

    // "Z" -> ground truth for the LM, from Orbiter itself and not from NASSP,
    // so a recorded instrument can be judged against the actual vehicle
    // rather than against itself. Needs no patch; works on stock NASSP.
    //
    //   "z <simt> <alt_ground> <alt_meanrad> <vspeed>
    //      <gs_local x y z> <angvel x y z> <pitch> <bank> <yaw>
    //      <thrust_N> <mass_kg>"
    //
    // Metres, m/s, rad and rad/s. Local-frame vectors are Orbiter's vessel
    // frame (left-handed: x right, y up, z forward); the vertical speed is the
    // horizon frame's y.
    if (verb == 'Z')
    {
        VESSEL *v = oapiGetVesselInterface(hVessel);
        VECTOR3 hz = _V(0, 0, 0), gs = _V(0, 0, 0), w = _V(0, 0, 0),
                th = _V(0, 0, 0), pos = _V(0, 0, 0);
        MATRIX3 rot;
        v->GetGroundspeedVector(FRAME_HORIZON, hz);
        v->GetGroundspeedVector(FRAME_LOCAL, gs);
        v->GetAngularVel(w);
        v->GetThrustVector(th);
        // The vessel-to-global rotation the IMU itself reads
        // (IMU::Timestep), and the position from the gravity reference,
        // so a recording can recover the platform's orientation in space.
        v->GetRotationMatrix(rot);
        OBJHANDLE ref = v->GetGravityRef();
        if (ref)
            v->GetRelativePos(ref, pos);
        sprintf(out, "z %.3f %.2f %.2f %.3f %.3f %.3f %.3f %.5f %.5f %.5f "
                     "%.5f %.5f %.5f %.1f %.1f "
                     "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f "
                     "%.1f %.1f %.1f\n",
                oapiGetSimTime(), v->GetAltitude(ALTMODE_GROUND),
                v->GetAltitude(ALTMODE_MEANRAD), hz.y, gs.x, gs.y, gs.z,
                w.x, w.y, w.z, v->GetPitch(), v->GetBank(), v->GetYaw(),
                length(th), v->GetMass(),
                rot.m11, rot.m12, rot.m13, rot.m21, rot.m22, rot.m23,
                rot.m31, rot.m32, rot.m33, pos.x, pos.y, pos.z);
        Reply(s, from, fromlen, out);
        return;
    }

    if (verb == 'G')
    {
        int state = panel.GetItemState(line);
        sprintf(out, "V %.80s %d\n", line, state);
        Reply(s, from, fromlen, out);
        return;
    }

    if (verb == 'S')
    {
        if (!sp)
        {
            sprintf(out, "E %.80s no-value\n", line);
            Reply(s, from, fromlen, out);
            return;
        }
        // "S <item> <value> [hold]". The hold field is optional so a hand-typed
        // command still works; the bridge sets it for momentary controls.
        int  value = atoi(sp + 1);
        bool hold  = false;
        char *sp2  = strchr(sp + 1, ' ');
        if (sp2)
        {
            hold = (atoi(sp2 + 1) != 0);
        }
        // "S <item> 3.25": a value with a decimal point is for a continuous
        // knob, set at full resolution through the patch (LMDD_SET_KNOB) and
        // answered with the value read back. On a stock LEM.dll, or for an
        // item that is not a continuous knob, it is rounded and set below.
        char *dot = strchr(sp + 1, '.');
        if (dot && (!sp2 || dot < sp2))
        {
            double knob = atof(sp + 1);
            if (display.Available() && display.Knob(line, true, &knob))
            {
                sprintf(out, "K %.80s %.4f\n", line, knob);
                Reply(s, from, fromlen, out);
                return;
            }
            value = (int)floor(atof(sp + 1) + 0.5);
        }
        bool ok = panel.SetItemState(line, value, hold);
        sprintf(out, ok ? "K %.80s %d\n" : "E %.80s\n", line, value);
        Reply(s, from, fromlen, out);
        return;
    }

    sprintf(out, "E bad-verb %c\n", verb);
    Reply(s, from, fromlen, out);
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
