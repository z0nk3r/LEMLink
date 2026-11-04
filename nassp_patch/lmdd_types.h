/***************************************************************************
  The wire between the patched LEM.dll and LEMLink.dll: message numbers and
  reply layouts for LEMDisplayDataConnector. See lemdisplaydata.h and patch.md.

  ONE header, compiled into BOTH DLLs. The plugin used to keep its own copy of
  the message enum, which is two copies of an ABI that nothing checks against
  each other - the classic way for one side to grow a message and the other to
  read the wrong one. apply_patch.py copies this file into NASSP's src_lm/, and
  LEMLink.cpp includes it from here.

  Plain C types only and no NASSP includes, so the plugin can take it without
  vendoring LEM.h.
 ***************************************************************************/

#ifndef LMDD_TYPES_H
#define LMDD_TYPES_H

/*
 * Message types. APPEND ONLY - never reorder or insert. The ordinal is what
 * crosses the DLL boundary, and one that moved would read a DSKY as an FDAI
 * without either side noticing.
 */
enum LMDDMessageType
{
	LMDD_GET_CW_LIGHT,        /* not answered any more; holds the ordinals below */
	LMDD_GET_CW_LIGHT_ROW,    /* a whole row of eight, packed two bits each */
	LMDD_GET_MISSION_TIMER,   /* mission timer, seconds */
	LMDD_GET_EVENT_TIMER,     /* event timer, seconds */
	LMDD_GET_METER,           /* a MeterSwitch's displayed value, by name */

	/* everything the panel draws that is not a meter. Each takes a
	 * caller-owned struct below in val1.pValue. */
	LMDD_GET_FDAI,
	LMDD_GET_XPOINTER,
	LMDD_GET_TAPE,
	LMDD_GET_DSKY,
	LMDD_GET_DEDA,
	LMDD_GET_AOT,

	/* the lamps the panel draws from subsystem logic, outside
	 * the CWEA grid, the DSKY and the DEDA. */
	LMDD_GET_PANEL_LAMPS,
	LMDD_GET_ATTITUDE,        /* the chain behind the FDAI ball */
	LMDD_GET_ERASABLE,        /* up to 16 words of AGC erasable memory */
	/* the legend lamps only the VC lights (X-pointer source and
	 * scale, tape mode, RCS He x10). */
	LMDD_GET_VC_LEGENDS,
	/* every lighting channel's level, for the pit's backlights */
	LMDD_GET_LIGHTING,
	/* a continuous knob's position as a double, in the same
	 * display units as the panel connector's whole numbers (0..8 for a
	 * lighting knob). The panel connector's int SetState() rounds, which
	 * gave a pit pot 9 settings. The one write in this connector: a knob is
	 * a control, not a display, so nothing overwrites it next timestep.
	 *   val1.pValue in the knob's registered name, val3.dValue in the value
	 *   (clamped by the knob, as a mouse drag is), val1.dValue out the knob's
	 *   value after the call, val2.bValue out false if no continuous knob has
	 *   that name. */
	LMDD_SET_KNOB,
};

/*
 * Every reply struct starts with `size`, which the caller sets to sizeof the
 * struct it is passing. The connector refuses a size it does not expect and
 * answers false, so a plugin built against an older layout gets "no display"
 * instead of having memory past the end of its struct written over.
 *
 * Needles are reported AS DRAWN: -1..+1 of full deflection, clamped where the
 * panel clamps, with the panel's own sign. Not the quantity behind them - a
 * needle's source changes with switches (an FDAI error needle can be reading
 * the rendezvous radar, the LGC or the AGS), and the pit wants the needle.
 */

/* LMDD_GET_FDAI. Rates: roll, pitch, yaw as AnimateFDAI turns them. Errors:
 * the same axes, full scale at 41 px as lempanel.cpp clamps them. Ball: the
 * displayed attitude in degrees - the ball slews at about 50 deg/s, so this
 * lags the IMU exactly as the instrument does. */
struct LMDDFdai
{
	int    size;
	int    which;      /* in: 0 = left (CDR), 1 = right (LMP) */
	int    powered;    /* out: 0 means the OFF flag is showing */
	int    reserved;
	double rate[3];
	double error[3];
	double ball[3];
};

/* LMDD_GET_XPOINTER. needle[0] is FORWARD (the horizontal bar),
 * needle[1] LATERAL (the vertical bar), -1..+1, full scale = 20 units of
 * CrossPointer's own velocity after its x10 switch. raw[] is that unclamped
 * velocity, in the same units. */
struct LMDDXPointer
{
	int    size;
	int    which;      /* in: 0 = left (CDR), 1 = right (LMP) */
	int    powered;
	int    fine;       /* 1 when the scale switch selects the x10 range */
	double needle[2];
	double raw[2];
};

/* LMDD_GET_TAPE. *_pos are tape POSITIONS - the pixel offset into NASSP's
 * tape bitmap, 0..8486 for range - which is what a motorised physical tape
 * needs to be driven to. range_m and rate_mps are the values the tape was
 * last commanded to show, before the tape's own slew limit. */
struct LMDDTape
{
	int    size;
	int    powered;
	int    monitor_on;   /* PowerSignalMonOn(): the tape's own warning flag */
	int    power_fail;
	int    signal_fail;
	int    timing_fail;
	double range_pos;
	double rate_pos;
	double range_m;
	double rate_mps;
};

/* LMDD_GET_DSKY. Digit strings exactly as NASSP holds them, a space for a
 * blank digit. The bits in `lamps` follow LMDD_DSKY_* below. */
struct LMDDDsky
{
	int      size;
	int      segment_power;
	int      status_power;
	unsigned lamps;
	char     prog[4];
	char     verb[4];
	char     noun[4];
	char     r1[8];
	char     r2[8];
	char     r3[8];
};

enum
{
	LMDD_DSKY_UPLINK    = 1 << 0,
	LMDD_DSKY_COMPACTY  = 1 << 1,
	LMDD_DSKY_NOATT     = 1 << 2,
	LMDD_DSKY_STBY      = 1 << 3,
	LMDD_DSKY_KEYREL    = 1 << 4,
	LMDD_DSKY_OPRERR    = 1 << 5,
	LMDD_DSKY_TEMP      = 1 << 6,
	LMDD_DSKY_GIMBAL    = 1 << 7,
	LMDD_DSKY_PROG      = 1 << 8,
	LMDD_DSKY_RESTART   = 1 << 9,
	LMDD_DSKY_TRACKER   = 1 << 10,
	LMDD_DSKY_VEL       = 1 << 11,
	LMDD_DSKY_ALT       = 1 << 12,
	LMDD_DSKY_NODAP     = 1 << 13,
	LMDD_DSKY_PRIODISP  = 1 << 14,
};

/* LMDD_GET_DEDA. Address and data as NASSP holds them. */
struct LMDDDeda
{
	int  size;
	int  powered;
	int  opr_err;
	char adr[4];
	char data[8];
};

/* LMDD_GET_AOT. counter_deg is what the reticle counter reads, 0..360,
 * computed the way PaintReticleAngle() draws it. shaft is the detent:
 * 0, 1, 2, 3, 4, 5 = 0, 60, 120, 180, -120, -60 degrees. */
struct LMDDAot
{
	int    size;
	int    shaft;
	int    dimmer;
	int    reserved;
	double counter_deg;
	double reticle_rad;
};

/* LMDD_GET_PANEL_LAMPS. One bit per lamp, LMDD_LAMP_* below, lit exactly
 * when the 2D panel would draw it lit: each bit is the condition from that
 * lamp's case in LEM::clbkPanelRedrawEvent, power, lighting bus and lamp
 * test included. apply_patch.py refuses to build if any of those conditions
 * has changed in NASSP, so the two cannot drift. */
struct LMDDLamps
{
	int      size;
	unsigned lit;
};

enum
{
	LMDD_LAMP_CONTACT_CDR    = 1 << 0,   /* AID_CONTACTLIGHT1, panel 1 */
	LMDD_LAMP_CONTACT_LMP    = 1 << 1,   /* AID_CONTACTLIGHT2 */
	LMDD_LAMP_RR_NO_TRACK    = 1 << 2,
	LMDD_LAMP_GLYCOL         = 1 << 3,   /* panel 2 component caution lamps */
	LMDD_LAMP_SUIT_FAN       = 1 << 4,
	LMDD_LAMP_CO2            = 1 << 5,
	LMDD_LAMP_H2O_SEP        = 1 << 6,
	LMDD_LAMP_DC_BUS         = 1 << 7,   /* panel 14 */
	LMDD_LAMP_BAT_FAULT      = 1 << 8,
	LMDD_LAMP_STAGE_SEQ_A    = 1 << 9,   /* panel 8 */
	LMDD_LAMP_STAGE_SEQ_B    = 1 << 10,
	/* the small red power-failure lamps beside the indicators */
	LMDD_LAMP_PF_XPTR_CDR    = 1 << 11,
	LMDD_LAMP_PF_THRUST      = 1 << 12,
	LMDD_LAMP_PF_DPS_PRESS   = 1 << 13,
	LMDD_LAMP_PF_RCS_PRESS   = 1 << 14,
	LMDD_LAMP_PF_RCS_QTY     = 1 << 15,
	LMDD_LAMP_PF_ECS_PRESS   = 1 << 16,
	LMDD_LAMP_PF_GLYCOL      = 1 << 17,
	LMDD_LAMP_PF_ECS_QTY     = 1 << 18,
	LMDD_LAMP_PF_XPTR_LMP    = 1 << 19,
	LMDD_LAMP_PF_TAPE        = 1 << 20,
	/* Not panel cases, so not generated: the master alarm as RenderMasterAlarm
	 * draws it, and the two lighting supplies the CWEA grid is drawn through.
	 * LEM_CWEA::RedrawLeft/Right draw a LightStatus of 1 lit only while
	 * IsLTGPowered(), or IsCWPWRLTGPowered() for C/W PWR at row 3 column 6,
	 * so LMDD_GET_CW_LIGHT_ROW alone is the latch, not the lamp. */
	LMDD_LAMP_MASTER_ALARM   = 1 << 21,
	LMDD_LAMP_CW_LTG         = 1 << 22,   /* gate for the CWEA grid */
	LMDD_LAMP_CW_PWR_LTG     = 1 << 23,   /* gate for C/W PWR alone */
	LMDD_LAMP_HAS_GATES      = 1 << 24,   /* always set: bits 21-23 are real */
	LMDD_LAMP_ENG_START      = 1 << 25,   /* EngineStartButton::LightLogic() */
	LMDD_LAMP_HAS_ENGSTART   = 1 << 26,   /* always set: bit 25 is real */
};

/* LMDD_GET_VC_LEGENDS. The legends the virtual cockpit lights and the 2D
 * panel never draws, one bit each, LMDD_LEGEND_* below. A bit is the
 * CONDITION: LEM::clbkVCRedrawEvent passes XP_LIT_ON for it rather than
 * XP_LIT_OFF. LEM::GetVCLegendBits() is generated by apply_patch.py from that
 * block of lemvc.cpp, so the two cannot drift. How bright it draws is
 * separate: max(numeric, 0.15) lit and 0.15 unlit, where numeric is
 * lca.GetNumericOutput() (the NUM dimmer channel, volts / 115), so a legend
 * is visibly lit only while numeric > 0.15. */
struct LMDDLegends
{
	int      size;
	unsigned lit;
	double   numeric;
};

enum
{
	LMDD_LEGEND_CDR_ELEV_RT  = 1 << 0,   /* X-pointer, panel 1: left edge, upper */
	LMDD_LEGEND_CDR_AZ_RT    = 1 << 1,   /* bottom edge, left */
	LMDD_LEGEND_CDR_LAT_VEL  = 1 << 2,   /* bottom edge, right */
	LMDD_LEGEND_CDR_FWD_VEL  = 1 << 3,   /* left edge, lower */
	LMDD_LEGEND_CDR_X1       = 1 << 4,   /* bottom edge, centre */
	LMDD_LEGEND_CDR_X10      = 1 << 5,
	LMDD_LEGEND_LMP_ELEV_RT  = 1 << 6,   /* X-pointer, panel 2, same places */
	LMDD_LEGEND_LMP_AZ_RT    = 1 << 7,
	LMDD_LEGEND_LMP_LAT_VEL  = 1 << 8,
	LMDD_LEGEND_LMP_FWD_VEL  = 1 << 9,
	LMDD_LEGEND_LMP_X1       = 1 << 10,
	LMDD_LEGEND_LMP_X10      = 1 << 11,
	LMDD_LEGEND_TAPE_ALT     = 1 << 12,  /* range/rate tape: ALT / ALT RATE */
	LMDD_LEGEND_TAPE_RANGE   = 1 << 13,  /* RANGE / RANGE RATE */
	LMDD_LEGEND_RCS_HE_X10   = 1 << 14,  /* RCS PRESS meter's x10, panel 2 */
};

/* LMDD_GET_LIGHTING. Each lighting channel's level, as the VC scales it in
 * the AID_LMVC_LIGHTING case of clbkVCRedrawEvent, which apply_patch.py
 * checks still holds each expression mirrored here:
 *   integral   lca.GetIntegralOutput(): INTGL LTG AC volts / 115, so the
 *              INTEGRAL knob spans 15-75 V (0.13-0.65) and LTG O'RIDE passes
 *              the whole bus (1.0). The VC adds the floods' fake ambient
 *              (floodRotaryValue) for its textures; that is NOT added here.
 *   numeric    lca.GetNumericOutput(): 20-110 V knob, bus on override, / 115
 *   anun       LtgORideAnunSwitch.Voltage() / 6: 2-5 V by knob, 6 on override
 *   dsky       dsky.Variable_250VAC_Output.Voltage() / 250
 *   flood_*    FloodLights.GetCDROutput() / GetLMPOutput() / GetSideOutput()
 *   ordeal     ordeal.LightingPower(): 1, 0.5 at DIM, 0 off or unpowered
 *   side_*     the switches that gate the integral on panels 8 and 11
 *              (LtgSidePanelsSwitch) and 12, 14 and 16 (SidePanelsSwitch) */
struct LMDDLighting
{
	int    size;
	double integral, numeric, anun, dsky;
	double flood_cdr, flood_lmp, flood_side, ordeal;
	int    side_8_11, side_12_14_16;
};

/* LMDD_GET_ATTITUDE. Every stage between the IMU and the FDAI ball, so the
 * ball can be judged against its source rather than against the local
 * horizon, which it never shows:
 *
 *   IMU gimbal angles --GASTA--> attitude (PGNS)  \  ATT MON picks one,
 *   AEA attitude                 (AGS)            /  ORDEAL adds pitch,
 *                                                     the ball slews to it
 *
 * lemsystems.cpp. All in degrees, folded to 0..360. gasta[] and ags[] are
 * roll, pitch, yaw - the order LMDDFdai.ball uses. imu[] is the gimbal
 * angles as IMU::GetTotalAttitude() returns them (x, y, z). */
struct LMDDAttitude
{
	int    size;
	int    imu_powered;
	int    imu_caged;
	int    gasta_powered;
	double imu[3];
	double gasta[3];
	double ags[3];
	double ordeal_pitch[2];   /* added to FDAI 1 (CDR) and 2 (LMP) pitch */
};

/* LMDD_GET_ERASABLE. Consecutive words of the LGC's erasable memory, read
 * with ApolloGuidance::GetErasable. `address` is flattened the way yaYUL
 * listings write it: bank * 0400 + offset, 0..03777 octal. Words are the raw
 * 15-bit ones'-complement contents. Read-only: nothing here writes the AGC.
 * Used to read what the LGC itself computed - FORVEL/LATVEL behind the
 * X-pointer, say - so an instrument can be told apart from its source. */
struct LMDDErasable
{
	int size;
	int address;              /* in: first word */
	int count;                /* in: 1..16 */
	int words[16];            /* out */
};

#endif /* LMDD_TYPES_H */
