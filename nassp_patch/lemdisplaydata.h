/***************************************************************************
This file is part of Project Apollo - NASSP

Connector exposing LM display data that is drawn by its own subsystem and is
therefore not reachable as a named panel item: the caution and warning lights,
the mission and event timers, and meter readings.

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

#if !defined(_PA_LEMDISPLAYDATA_H)
#define _PA_LEMDISPLAYDATA_H

#include "lemconnector.h"
#include "lmdd_types.h"

///
/// \ingroup Connectors
/// \brief Read-only access to LM displays that are not panel items.
///
/// MFD_PANEL_INTERFACE reaches anything registered with PanelSwitches, which
/// covers every switch, breaker and talkback. It does not cover displays that
/// a subsystem draws for itself: the caution and warning lights come straight
/// out of LEM_CWEA::LightStatus, the timers out of MissionTimer, and a meter's
/// needle is a double behind GetDisplayValue() that the int GetState() on
/// PanelSwitchItem cannot carry.
///
/// External panel hardware needs those three, and everything else the panel
/// draws that is not a meter: the FDAI needles and ball, the X-pointers, the
/// range and rate tapes, the DSKY, the DEDA and the AOT counter. Displays are
/// read-only, because a display is an output of the simulated systems and
/// writing one would only be overwritten on the next timestep. The one write
/// is LMDD_SET_KNOB: a continuous knob is a control, and the panel connector
/// can only set it in whole display units.
///
/// The message numbers and reply layouts live in lmdd_types.h, which the
/// plugin compiles too - one definition of the wire, not two.
///
class LEMDisplayDataConnector : public LEMConnector
{
public:
	LEMDisplayDataConnector(LEM *l);
	~LEMDisplayDataConnector();

	bool ReceiveMessage(Connector *from, ConnectorMessage &m);
};

#endif // _PA_LEMDISPLAYDATA_H
