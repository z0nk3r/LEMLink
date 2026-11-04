/***************************************************************************
This file is part of Project Apollo - NASSP

Connector exposing LM display data. See lemdisplaydata.h.

Project Apollo is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

Project Apollo is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Project Apollo; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

See http://nassp.sourceforge.net/license/ for more details.

**************************** Revision History ****************************/

// The prerequisites LEM.h assumes, in the order NASSP's own LM sources
// include them (lemconnector.cpp, lm_cwea.cpp). LEM.h is not self-contained:
// without these first, FDAI.h fails on e_object and ORDEAL.h on ToggleSwitch.
#include "Orbitersdk.h"
#include "soundlib.h"
#include "toggleswitch.h"
#include "apolloguidance.h"
#include "LEMcomputer.h"
#include "LEM.h"
#include "lemdisplaydata.h"

#include <math.h>
#include <string.h>

namespace
{
	double Clamp1(double v)
	{
		return v > 1.0 ? 1.0 : (v < -1.0 ? -1.0 : v);
	}

	/// An angle in degrees, folded into 0..360. The FDAI ball's `now` can sit
	/// a turn or more away from its true value - RotateBall() adds 2 pi to
	/// take the short way round - so it has to be folded before a pit gauge
	/// can use it.
	double Fold360(double deg)
	{
		deg = fmod(deg, 360.0);
		return deg < 0.0 ? deg + 360.0 : deg;
	}

	/// Copies a NASSP display string into a fixed reply field, always
	/// terminated, never reading past the source array.
	void CopyText(char *dst, size_t dstlen, const char *src, size_t srclen)
	{
		size_t n = 0;
		while (n < srclen && n + 1 < dstlen && src[n])
		{
			dst[n] = src[n];
			n++;
		}
		dst[n] = 0;
	}
}

///
/// Checks the reply struct a message points at. Every reply struct starts with
/// its own size, set by the caller; a mismatch means the plugin was built
/// against a different lmdd_types.h, and writing into its struct would walk
/// off the end of it. So it is refused, and val2 says so.
///
#define LMDD_REPLY(T, name)                                              \
	T *name = static_cast<T *>(m.val1.pValue);                            \
	m.val2.bValue = (name != 0 && name->size == (int)sizeof(T));          \
	if (!m.val2.bValue) { return true; }

LEMDisplayDataConnector::LEMDisplayDataConnector(LEM *l) : LEMConnector(l)
{
	type = LM_DISPLAY_DATA;
}

LEMDisplayDataConnector::~LEMDisplayDataConnector()
{
}

bool LEMDisplayDataConnector::ReceiveMessage(Connector *from, ConnectorMessage &m)
{
	if (m.destination != type)
	{
		return false;
	}

	if (!OurVessel)
	{
		return false;
	}

	LMDDMessageType messageType = (LMDDMessageType) m.messageType;

	switch (messageType)
	{
	case LMDD_GET_CW_LIGHT_ROW:
		//
		// val1 in: row. val1 out: the row's eight lights, two bits each,
		// column 0 in the low bits.
		//
		// A whole row per message because a panel driver wants all forty
		// lights every update, and forty round trips per update is forty
		// times the traffic for no more information.
		//
		{
			int row = m.val1.iValue;
			int packed = 0;
			for (int col = 0; col < 8; col++)
			{
				int state = OurVessel->CWEA.GetCWLightStatus(row, col);
				packed |= (state & 3) << (col * 2);
			}
			m.val1.iValue = packed;
		}
		return true;

	case LMDD_GET_MISSION_TIMER:
		m.val1.dValue = OurVessel->MissionTimerDisplay.GetTime();
		return true;

	case LMDD_GET_EVENT_TIMER:
		m.val1.dValue = OurVessel->EventTimerDisplay.GetTime();
		return true;

	case LMDD_SET_KNOB:
		//
		// A continuous knob at full resolution (lmdd_types.h). Through the
		// same SwitchTo() a mouse drag uses, so the clamp, the click sound
		// and everything downstream of the knob behave as they do by hand.
		//
		{
			ContinuousSwitch *knob = OurVessel->MainPanel.GetContinuousByName(
			                             static_cast<char *>(m.val1.pValue));
			m.val2.bValue = (knob != 0);
			if (knob)
			{
				knob->SetDisplayValue(m.val3.dValue);
			}
			m.val1.dValue = knob ? knob->GetValue() : 0.0;
		}
		return true;

	case LMDD_GET_METER:
		//
		// val1 in: the meter's registered name. val1 out: its displayed
		// value. val2 out: false if there is no meter by that name, so a
		// caller can tell a missing meter from one reading zero.
		//
		{
			MeterSwitch *meter = OurVessel->MainPanel.GetMeterByName(
			                         static_cast<char *>(m.val1.pValue));
			m.val2.bValue = (meter != 0);
			m.val1.dValue = meter ? meter->GetDisplayValue() : 0.0;
		}
		return true;

	case LMDD_GET_FDAI:
		//
		// The needles as drawn. GetFDAIRatesErrors() is the one place the
		// panel and the VC both take their inputs from (see patch.md), so
		// this cannot disagree with either. The axis order and signs are
		// AnimateFDAI's: rates come in as (pitch, yaw, roll) and are
		// turned as -z, -x, +y; errors as +x, -y, +z.
		//
		{
			LMDD_REPLY(LMDDFdai, r);
			bool right = (r->which != 0);
			FDAI &fdai = right ? OurVessel->fdaiRight : OurVessel->fdaiLeft;
			VECTOR3 rates, errors;
			OurVessel->GetFDAIRatesErrors(right, rates, errors);

			r->powered = fdai.IsPowered() ? 1 : 0;
			r->rate[0] = Clamp1(-rates.z);
			r->rate[1] = Clamp1(-rates.x);
			r->rate[2] = Clamp1(rates.y);
			r->error[0] = Clamp1(errors.x / 41.0);
			r->error[1] = Clamp1(-errors.y / 41.0);
			r->error[2] = Clamp1(errors.z / 41.0);

			// SetAttitude() stores roll as -x into now.y, pitch into now.z
			// and yaw into now.x; undo that so the ball reads as attitude.
			VECTOR3 ball = fdai.GetBallAttitude();
			r->ball[0] = Fold360(-ball.y * DEG);
			r->ball[1] = Fold360(ball.z * DEG);
			r->ball[2] = Fold360(ball.x * DEG);
		}
		return true;

	case LMDD_GET_XPOINTER:
		//
		// CrossPointer::Timestep() already leaves its velocities in needle
		// units, 20 = full deflection, with the x10 switch applied. Both
		// views draw vel/20 (the panel at 3 px per unit over +/-60, the VC as
		// vel/40 + 0.5), so the drawn needle is simply that, clamped.
		//
		{
			LMDD_REPLY(LMDDXPointer, r);
			bool right = (r->which != 0);
			CrossPointer &cp = right ? OurVessel->crossPointerRight
			                         : OurVessel->crossPointerLeft;
			ToggleSwitch &scale = right ? OurVessel->RightXPointerSwitch
			                            : OurVessel->LeftXPointerSwitch;
			double vx = 0.0, vy = 0.0;
			cp.GetVelocities(vx, vy);
			r->powered = cp.IsPowered() ? 1 : 0;
			r->fine = scale.IsDown() ? 1 : 0;
			r->raw[0] = vx;
			r->raw[1] = vy;
			r->needle[0] = Clamp1(vx / 20.0);
			r->needle[1] = Clamp1(vy / 20.0);
		}
		return true;

	case LMDD_GET_TAPE:
		//
		// The tape POSITIONS, which is what a physical tape is driven to.
		// They come out of LEM_RadarTape::Timestep() every step, slew-
		// limited like the real tape, whether or not anything is drawing it.
		//
		{
			LMDD_REPLY(LMDDTape, r);
			LEM_RadarTape &tape = OurVessel->RadarTape;
			r->powered = tape.IsPowered() ? 1 : 0;
			r->monitor_on = tape.PowerSignalMonOn() ? 1 : 0;
			r->power_fail = tape.PowerFailure() ? 1 : 0;
			r->signal_fail = tape.SignalFailure() ? 1 : 0;
			r->timing_fail = tape.TimingFailure() ? 1 : 0;
			r->range_pos = tape.dispRange;
			r->rate_pos = tape.dispRate;
			r->range_m = tape.reqRange;
			r->rate_mps = tape.reqRate;
		}
		return true;

	case LMDD_GET_DSKY:
		{
			LMDD_REPLY(LMDDDsky, r);
			DSKY &d = OurVessel->dsky;
			// What dsky.cpp draws on (friend, NASSP build 47): status lamps
			// while IsStatusPowered(), digits and COMP ACTY while
			// IsSegmentPowered() and the EL supply is not switched off.
			r->segment_power = (d.IsSegmentPowered() && !d.ELOff) ? 1 : 0;
			r->status_power = d.IsStatusPowered() ? 1 : 0;
			CopyText(r->prog, sizeof(r->prog), d.GetProg(), 3);
			CopyText(r->verb, sizeof(r->verb), d.GetVerb(), 3);
			CopyText(r->noun, sizeof(r->noun), d.GetNoun(), 3);
			CopyText(r->r1, sizeof(r->r1), d.GetR1(), 7);
			CopyText(r->r2, sizeof(r->r2), d.GetR2(), 7);
			CopyText(r->r3, sizeof(r->r3), d.GetR3(), 7);
			unsigned lamps = 0;
			if (d.UplinkLit())     lamps |= LMDD_DSKY_UPLINK;
			if (d.CompActyLit())   lamps |= LMDD_DSKY_COMPACTY;
			if (d.NoAttLit())      lamps |= LMDD_DSKY_NOATT;
			if (d.StbyLit())       lamps |= LMDD_DSKY_STBY;
			if (d.KbRelLit())      lamps |= LMDD_DSKY_KEYREL;
			if (d.OprErrLit())     lamps |= LMDD_DSKY_OPRERR;
			if (d.TempLit())       lamps |= LMDD_DSKY_TEMP;
			if (d.GimbalLockLit()) lamps |= LMDD_DSKY_GIMBAL;
			if (d.ProgLit())       lamps |= LMDD_DSKY_PROG;
			if (d.RestartLit())    lamps |= LMDD_DSKY_RESTART;
			if (d.TrackerLit())    lamps |= LMDD_DSKY_TRACKER;
			if (d.VelLit())        lamps |= LMDD_DSKY_VEL;
			if (d.AltLit())        lamps |= LMDD_DSKY_ALT;
			if (d.NoDAPLit())      lamps |= LMDD_DSKY_NODAP;
			if (d.PrioDispLit())   lamps |= LMDD_DSKY_PRIODISP;
			r->lamps = lamps;
		}
		return true;

	case LMDD_GET_DEDA:
		{
			LMDD_REPLY(LMDDDeda, r);
			LEM_DEDA &d = OurVessel->deda;
			r->powered = d.IsPowered() ? 1 : 0;
			r->opr_err = d.OprErrLit() ? 1 : 0;
			CopyText(r->adr, sizeof(r->adr), d.Adr, sizeof(d.Adr));
			CopyText(r->data, sizeof(r->data), d.Data, sizeof(d.Data));
		}
		return true;

	case LMDD_GET_AOT:
		//
		// The counter as PaintReticleAngle() draws it: -OpticsReticle in
		// degrees, folded into 0..360.
		//
		{
			LMDD_REPLY(LMDDAot, r);
			LMOptics &o = OurVessel->optics;
			r->shaft = o.OpticsShaft;
			r->dimmer = o.RetDimmer;
			r->reticle_rad = o.OpticsReticle;
			r->counter_deg = Fold360(-o.OpticsReticle * DEG);
		}
		return true;

	case LMDD_GET_PANEL_LAMPS:
		//
		// LEM::GetPanelLampBits() is generated by apply_patch.py from the
		// lamps' own cases in clbkPanelRedrawEvent, so these bits are the
		// panel's conditions, not a copy of them.
		//
		{
			LMDD_REPLY(LMDDLamps, r);
			r->lit = OurVessel->GetPanelLampBits();
			LEM_CWEA &cw = OurVessel->CWEA;
			if (cw.GetMasterAlarm())     r->lit |= LMDD_LAMP_MASTER_ALARM;
			if (cw.IsLTGPowered())       r->lit |= LMDD_LAMP_CW_LTG;
			if (cw.IsCWPWRLTGPowered())  r->lit |= LMDD_LAMP_CW_PWR_LTG;
			r->lit |= LMDD_LAMP_HAS_GATES;
			// Build 47: START is momentary and its lamp shows SCS relay K15,
			// which START sets and STOP clears - not the button. Ask the
			// button's own LightLogic() (friend) rather than copy it.
			if (OurVessel->ManualEngineStart.LightLogic()) r->lit |= LMDD_LAMP_ENG_START;
			r->lit |= LMDD_LAMP_HAS_ENGSTART;
		}
		return true;

	case LMDD_GET_VC_LEGENDS:
		//
		// LEM::GetVCLegendBits() is generated by apply_patch.py from the
		// legend block of clbkVCRedrawEvent in lemvc.cpp: the VC's own
		// conditions, run with XP_LIT_ON recorded as a bit. The level the VC
		// draws them at comes from the numerics dimmer, reported beside it.
		//
		{
			LMDD_REPLY(LMDDLegends, r);
			r->lit = OurVessel->GetVCLegendBits();
			r->numeric = OurVessel->lca.GetNumericOutput();
		}
		return true;

	case LMDD_GET_LIGHTING:
		//
		// Each channel as the VC scales it (AID_LMVC_LIGHTING in lemvc.cpp;
		// apply_patch.py refuses to build if one of these expressions has
		// gone from there). The VC's flood ambient is left out of integral.
		//
		{
			LMDD_REPLY(LMDDLighting, r);
			LEM *l = OurVessel;
			r->integral = l->lca.GetIntegralOutput();
			r->numeric = l->lca.GetNumericOutput();
			r->anun = l->LtgORideAnunSwitch.Voltage() / 6.0;
			r->dsky = l->dsky.Variable_250VAC_Output.Voltage() / 250.0;
			r->flood_cdr = l->FloodLights.GetCDROutput();
			r->flood_lmp = l->FloodLights.GetLMPOutput();
			r->flood_side = l->FloodLights.GetSideOutput();
			r->ordeal = l->ordeal.LightingPower();
			r->side_8_11 = l->LtgSidePanelsSwitch.GetState() == TOGGLESWITCH_UP;
			r->side_12_14_16 = l->SidePanelsSwitch.GetState() == TOGGLESWITCH_UP;
		}
		return true;

	case LMDD_GET_ERASABLE:
		{
			LMDD_REPLY(LMDDErasable, r);
			int n = r->count < 1 ? 1 : (r->count > 16 ? 16 : r->count);
			for (int i = 0; i < n; i++)
			{
				int a = (r->address + i) & 03777;
				r->words[i] = OurVessel->agc.GetErasable(a >> 8, a & 0377) & 077777;
			}
			r->count = n;
		}
		return true;

	case LMDD_GET_ATTITUDE:
		//
		// What lemsystems.cpp hands each FDAI: gasta (ATT MON PGNS) or aea
		// (AGS), plus the ORDEAL pitch offset. The ball then slews to it.
		//
		{
			LMDD_REPLY(LMDDAttitude, r);
			LEM &l = *OurVessel;
			VECTOR3 v = l.imu.GetTotalAttitude();
			r->imu_powered = l.imu.IsPowered() ? 1 : 0;
			r->imu_caged = l.imu.IsCaged() ? 1 : 0;
			r->imu[0] = Fold360(v.x * DEG);
			r->imu[1] = Fold360(v.y * DEG);
			r->imu[2] = Fold360(v.z * DEG);
			r->gasta_powered = l.gasta.IsPowered() ? 1 : 0;
			v = l.gasta.GetTotalAttitude();
			r->gasta[0] = Fold360(v.x * DEG);
			r->gasta[1] = Fold360(v.y * DEG);
			r->gasta[2] = Fold360(v.z * DEG);
			v = l.aea.GetTotalAttitude();
			r->ags[0] = Fold360(v.x * DEG);
			r->ags[1] = Fold360(v.y * DEG);
			r->ags[2] = Fold360(v.z * DEG);
			r->ordeal_pitch[0] = l.ordeal.GetFDAI1PitchAngle() * DEG;
			r->ordeal_pitch[1] = l.ordeal.GetFDAI2PitchAngle() * DEG;
		}
		return true;
	}

	return false;
}
