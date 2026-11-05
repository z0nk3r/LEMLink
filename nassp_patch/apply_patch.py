#!/usr/bin/env python3
"""Apply the local LM display-data patch to a NASSP source tree.

    python3 apply_patch.py <ProjectApollo-dir>

<ProjectApollo-dir> is NASSP's Orbitersdk/samples/ProjectApollo, at the
revision ../vendor/REVISION pins. Edits it in place, so point it at a copy.

Every edit is anchored on exact text, and every anchor must appear exactly
once: an edit that matches nothing, or matches twice, stops the whole run
before a single file is written. patch.md used to describe these edits in
prose for a human to make by hand, which is how an edit gets applied to the
wrong one of two near-identical lines and nobody knows.

The FDAI edit is the one with teeth. The panel and the VC each compute the
needle inputs inline - four copies, left and right, 2D and VC - and this moves
them into one helper so the connector serves exactly what is drawn. Before it
removes each copy it checks that copy IS the helper's body, with only the two
switch names differing. If NASSP ever changes one of the four, the patch
refuses rather than quietly changing what the panel draws.

NASSP's sources are ISO-8859, so everything is read and written as latin-1.
"""
import re
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
NEW_FILES = ("lmdd_types.h", "lemdisplaydata.h", "lemdisplaydata.cpp")

# The shared body of the four FDAI blocks, with the two switches that differ
# between left and right written as RATEERR and ATTMON.
FDAI_BODY = """
if (RATEERR.GetState() == 1)
{
	if (RR.IsPowered()) {
		if (ShiftTruSwitch.IsUp())
		{
			errors.z = RR.GetRadarTrunnionPos() * 41 / (50 * RAD);
			errors.y = RR.GetRadarShaftPos() * 41 / (50 * RAD);
			errors.x = 0.0;
		}
		else
		{
			errors.z = RR.GetRadarTrunnionPos() * 41 / (5 * RAD);
			errors.y = RR.GetRadarShaftPos() * 41 / (5 * RAD);
			errors.x = 0.0;
		}
	}
	else
	{
		errors = _V(0, 0, 0);
	}
}
else
{
	if (ATTMON.IsUp())	//PGNS
	{
		errors = _V(atca.lgc_err_x, atca.lgc_err_y, atca.lgc_err_z);
	}
	else							//AGS
	{
		VECTOR3 aeaerror = aea.GetAttitudeError();
		errors = _V(aeaerror.x, -aeaerror.y, -aeaerror.z)*DEG*41.0 / 15.0;

		if (DeadBandSwitch.IsUp())
		{
			errors *= 15.0 / 14.4;
		}
		else
		{
			errors *= 15.0 / 1.7;
		}
	}
}

if (RateScaleSwitch.IsUp())
{
	rates = rga.GetRates() / (25.0*RAD);
}
else
{
	rates = rga.GetRates() / (5.0*RAD);
}
"""

HELPER = """
//
// LEMLINK LOCAL PATCH. The FDAI needle inputs, for either FDAI.
//
// This used to be written out four times, inline in the panel and VC redraw
// code for each FDAI, which meant it only ran when something was drawing the
// instrument. LEMDisplayDataConnector needs the same numbers whether or not
// anything is on screen, so all four draw sites call this instead and the
// connector does too: what the pit reads is what NASSP draws, by
// construction. apply_patch.py checked each removed copy was this body.
//
// Errors come back in panel pixels (41 = full deflection) and rates as a
// fraction of full scale, both UNCLAMPED - the 2D panel clamps errors before
// PaintMe(), the VC hands them straight to AnimateFDAI(), and each call site
// keeps doing exactly what it did.
//
void LEM::GetFDAIRatesErrors(bool right, VECTOR3 &rates, VECTOR3 &errors)
{
	ToggleSwitch &rateErrMon = right ? RightRateErrorMonSwitch : RateErrorMonSwitch;
	ToggleSwitch &attMon = right ? RightAttitudeMonSwitch : AttitudeMonSwitch;
BODY}

"""


class Patch:
    def __init__(self, root):
        self.root = Path(root)
        self.text = {}

    def _load(self, rel):
        if rel not in self.text:
            path = self.root / rel
            if not path.exists():
                sys.exit(f"apply_patch: {path} not found - is this a "
                         f"ProjectApollo directory?")
            raw = path.read_bytes().decode("latin-1")
            if "\r\n" in raw:
                sys.exit(f"apply_patch: {rel} has CRLF line endings; this "
                         f"patch was written against LF sources")
            self.text[rel] = raw
        return self.text[rel]

    def replace(self, rel, old, new, what):
        s = self._load(rel)
        n = s.count(old)
        if n != 1:
            sys.exit(f"apply_patch: {rel}: {what}: anchor found {n} times, "
                     f"want exactly 1")
        self.text[rel] = s.replace(old, new)

    def after(self, rel, anchor, added, what):
        self.replace(rel, anchor, anchor + added, what)

    def fdai_site(self, rel, rate_sw, att_sw, what):
        """Replace one inline FDAI block with a call, after proving it is
        the helper's body.

        The switch test is matched on a word boundary: the left switch's name
        is a suffix of the right one's (RateErrorMonSwitch inside
        RightRateErrorMonSwitch), so a plain substring search mixes the two
        FDAIs up. A block counts only if the rate computation follows it -
        the same switch is tested elsewhere for other reasons.
        """
        s = self._load(rel)
        start_re = re.compile(r"(?<![A-Za-z_])" + re.escape(rate_sw)
                              + r"\.GetState\(\) == 1")
        end_pat = "rates = rga.GetRates() / (5.0*RAD);"
        blocks = []
        for m in start_re.finditer(s):
            j = s.find(end_pat, m.start())
            if j < 0 or j - m.start() > 3000:
                continue
            close = s.find("}", j)
            line_start = s.rfind("\n", 0, m.start()) + 1
            blocks.append((line_start, m.start(), close))
        if len(blocks) != 1:
            sys.exit(f"apply_patch: {rel}: {what}: found {len(blocks)} FDAI "
                     f"blocks, want exactly 1")
        line_start, i, close = blocks[0]
        # Leading whitespace only. Taking everything up to the match would
        # include the "if (" in front of the switch name.
        indent = re.match(r"[ \t]*", s[line_start:i]).group(0)
        block = s[line_start:close + 1]
        canon = block.replace(rate_sw, "RATEERR").replace(att_sw, "ATTMON")
        if norm(canon) != norm(FDAI_BODY):
            sys.exit(f"apply_patch: {rel}: {what}: the inline block differs "
                     f"from the helper's body - NASSP changed it, and the "
                     f"refactor would change behaviour")
        side = "true" if rate_sw.startswith("Right") else "false"
        call = f"{indent}GetFDAIRatesErrors({side}, rates, errors);"
        self.text[rel] = s[:line_start] + call + s[close + 1:]

    def write(self):
        for rel, s in self.text.items():
            (self.root / rel).write_bytes(s.encode("latin-1"))
        for name in NEW_FILES:
            shutil.copy(HERE / name, self.root / "src_lm" / name)


# The lamps LEM::clbkPanelRedrawEvent draws from subsystem logic: bit name in
# lmdd_types.h, and the panel area whose case decides it. Their conditions are
# EXTRACTED from those cases, never typed here, so the connector serves
# exactly what the panel draws.
LAMPS = [
    ("LMDD_LAMP_CONTACT_CDR", "AID_CONTACTLIGHT1"),
    ("LMDD_LAMP_CONTACT_LMP", "AID_CONTACTLIGHT2"),
    ("LMDD_LAMP_RR_NO_TRACK", "AID_RR_NOTRACK"),
    ("LMDD_LAMP_GLYCOL", "AID_GLYCOL_LIGHT"),
    ("LMDD_LAMP_SUIT_FAN", "AID_SUITFAN_LIGHT"),
    ("LMDD_LAMP_CO2", "AID_CO2_LIGHT"),
    ("LMDD_LAMP_H2O_SEP", "AID_H2OSEP_LIGHT"),
    ("LMDD_LAMP_DC_BUS", "AID_DC_BUS_LIGHT"),
    ("LMDD_LAMP_BAT_FAULT", "AID_BAT_FAULT_LIGHT"),
    ("LMDD_LAMP_STAGE_SEQ_A", "AID_SEQ_LIGHT1"),
    ("LMDD_LAMP_STAGE_SEQ_B", "AID_SEQ_LIGHT2"),
    ("LMDD_LAMP_PF_XPTR_CDR", "AID_LEM_PWRFAIL_XPTRCDR"),
    ("LMDD_LAMP_PF_THRUST", "AID_LEM_PWRFAIL_THRUST"),
    ("LMDD_LAMP_PF_DPS_PRESS", "AID_LEM_PWRFAIL_DPSPRESS"),
    ("LMDD_LAMP_PF_RCS_PRESS", "AID_LEM_PWRFAIL_RCSPRESS"),
    ("LMDD_LAMP_PF_RCS_QTY", "AID_LEM_PWRFAIL_RCSQUAN"),
    ("LMDD_LAMP_PF_ECS_PRESS", "AID_LEM_PWRFAIL_ECSPRESS"),
    ("LMDD_LAMP_PF_GLYCOL", "AID_LEM_PWRFAIL_GLYCOL"),
    ("LMDD_LAMP_PF_ECS_QTY", "AID_LEM_PWRFAIL_ECSQUAN"),
    ("LMDD_LAMP_PF_XPTR_LMP", "AID_LEM_PWRFAIL_XPTRLMP"),
    ("LMDD_LAMP_PF_TAPE", "AID_LEM_PWRFAIL_RNGALTRATE"),
]


def if_conditions(block):
    """The condition of every `if (...)` in a block, parentheses balanced."""
    out = []
    for m in re.finditer(r"\bif\s*\(", block):
        depth, i = 1, m.end()
        while depth:
            depth += {"(": 1, ")": -1}.get(block[i], 0)
            i += 1
        out.append(re.sub(r"\s+", " ", block[m.end():i - 1]).strip())
    return out


def lamp_condition(redraw, aid):
    """When the 2D panel draws this lamp lit, as one C expression.

    Two shapes exist, and anything else stops the patch: one `if` whose
    branch draws the lit lamp, and CO2's outer lighting-bus test around two
    lit branches (`if (...) {...} else if (...) {...}`)."""
    m = re.search(rf"\bcase\s+{aid}\s*:(.*?)\breturn\s+true\s*;", redraw, re.S)
    if not m:
        sys.exit(f"apply_patch: no `case {aid}:` in LEM::clbkPanelRedrawEvent")
    conds = if_conditions(m.group(1))
    if len(conds) == 1:
        return conds[0]
    if len(conds) == 3 and "else if" in m.group(1):
        return f"({conds[0]}) && (({conds[1]}) || ({conds[2]}))"
    sys.exit(f"apply_patch: `case {aid}:` has an unexpected shape "
             f"({len(conds)} conditions) - read it and update LAMPS")


# The legends the VC lights and the 2D panel never draws: LMDD_LEGEND_* bit
# name, and the VC material clbkVCRedrawEvent lights for it. Their conditions
# are the VC's own block, extracted whole by vc_legend_block(), never typed.
LEGENDS = [
    ("LMDD_LEGEND_CDR_ELEV_RT", "VC_MAT_Panel1_Bulb_ELEV_RT"),
    ("LMDD_LEGEND_CDR_AZ_RT", "VC_MAT_Panel1_Bulb_AZ_RT"),
    ("LMDD_LEGEND_CDR_LAT_VEL", "VC_MAT_Panel1_Bulb_LAT_VEL"),
    ("LMDD_LEGEND_CDR_FWD_VEL", "VC_MAT_Panel1_Bulbs_FWD_VEL"),
    ("LMDD_LEGEND_CDR_X1", "VC_MAT_Panel1_Bulb_X1"),
    ("LMDD_LEGEND_CDR_X10", "VC_MAT_Panel1_Bulb_X10"),
    ("LMDD_LEGEND_LMP_ELEV_RT", "VC_MAT_Panel2_Bulb_ELEV_RT"),
    ("LMDD_LEGEND_LMP_AZ_RT", "VC_MAT_Panel2_Bulb_AZ_RT"),
    ("LMDD_LEGEND_LMP_LAT_VEL", "VC_MAT_Panel2_Bulb_LAT_VEL"),
    ("LMDD_LEGEND_LMP_FWD_VEL", "VC_MAT_Panel2_Bulbs_FWD_VEL"),
    ("LMDD_LEGEND_LMP_X1", "VC_MAT_Panel2_Bulb_X1"),
    ("LMDD_LEGEND_LMP_X10", "VC_MAT_Panel2_Bulb_X10"),
    ("LMDD_LEGEND_TAPE_ALT", "VC_MAT_Panel1_Tapemeter_AltAltRate"),
    ("LMDD_LEGEND_TAPE_RANGE", "VC_MAT_Panel1_Tapemeter_RangeRangeRate"),
    ("LMDD_LEGEND_RCS_HE_X10", "VC_MAT_RCS_HE_PRESS_x10"),
]

LEGEND_START = "#define XP_LIT_ON  (std::max)(lca.GetNumericOutput(), 0.15)\n#define XP_LIT_OFF  0.15\n"
LEGEND_CALL = re.compile(r"SetVCLighting\(vcidx, (VC_MAT_\w+), MAT_EMISSION, XP_LIT_(?:ON|OFF), 1\);")


def vc_legend_block(vc):
    """The legend block of clbkVCRedrawEvent, checked to be safe to rerun.

    It runs from the XP_LIT_ON/XP_LIT_OFF defines to the case's `return
    true;`. The copy is executed in a reader, so every statement in it must be
    one of the SetVCLighting(...) calls that light a legend, and the legends
    it lights must be exactly LEGENDS: anything else NASSP adds there - a
    power draw, a state change, a new legend - stops the patch rather than
    running in the connector or going unreported."""
    n = vc.count(LEGEND_START)
    if n != 1:
        sys.exit(f"apply_patch: lemvc.cpp: legend defines found {n} times, want 1")
    start = vc.index(LEGEND_START) + len(LEGEND_START)
    end = vc.index("return true;", start)
    block = vc[start:end].rstrip() + "\n"
    code = re.sub(r"//[^\n]*", "", block)
    rest = LEGEND_CALL.sub("", code)
    if ";" in rest:
        stmt = rest[:rest.index(";") + 1].strip().splitlines()[-1].strip()
        sys.exit(f"apply_patch: lemvc.cpp legend block has a statement that is "
                 f"not a legend: `{stmt}` - read it and update LEGENDS")
    used = set(LEGEND_CALL.findall(code))
    want = {mat for _, mat in LEGENDS}
    if used != want:
        sys.exit(f"apply_patch: lemvc.cpp legend block lights {sorted(used - want)} "
                 f"beyond LEGENDS and misses {sorted(want - used)}")
    return block


def legend_helper(block):
    """LEM::GetVCLegendBits(): the VC's block, run against a recorder.

    A local named SetVCLighting hides the member for the copied block, so
    each call records instead of lighting a mesh. XP_LIT_ON and XP_LIT_OFF are
    redefined to 1 and 0 for the copy, which makes the recorded level the
    condition itself; how bright the VC draws it is the connector's business."""
    cases = "\n".join(f"\t\tcase {mat}: lit |= {bit}; break;"
                      for bit, mat in LEGENDS)
    return ("\n//\n// LEMLINK LOCAL PATCH. Which legends clbkVCRedrawEvent lights, one bit each\n"
            "// (LMDD_LEGEND_*). Generated by apply_patch.py from the legend block of\n"
            "// clbkVCRedrawEvent - do not edit by hand.\n//\n"
            "unsigned LEM::GetVCLegendBits()\n{\n\tunsigned lit = 0;\n"
            "\tconst UINT vcidx = 0;\n"
            "\tauto SetVCLighting = [&lit](UINT, int mat, auto, double level, int) {\n"
            "\t\tif (level < 0.5) return;\n\t\tswitch (mat) {\n"
            + cases + "\n\t\t}\n\t};\n"
            "#undef XP_LIT_ON\n#undef XP_LIT_OFF\n#define XP_LIT_ON 1.0\n#define XP_LIT_OFF 0.0\n"
            + block +
            "#undef XP_LIT_ON\n#undef XP_LIT_OFF\n\treturn lit;\n}\n")


# What LMDD_GET_LIGHTING mirrors from the VC's AID_LMVC_LIGHTING case, as
# (what, regex). The connector cannot run that case - it lights meshes - so it
# restates each channel, and these keep the restatement honest: if NASSP
# rewrites how the VC scales a channel, the patch stops instead of reporting
# a level the VC no longer draws.
LIGHTING_SOURCES = [
    ("integral", r"lca\.GetIntegralOutput\(\) \+ floodRotaryValue"),
    ("numeric", r"NumericLights_LMVC, MAT_LIGHT, lca\.GetNumericOutput\(\)"),
    ("annunciator", r"LtgORideAnunSwitch\.Voltage\(\) / 6\.0"),
    ("DSKY", r"dsky\.Variable_250VAC_Output\.Voltage\(\) / 250\.0"),
    ("CDR flood", r"floodLight_Left->SetIntensity\(FloodLights\.GetCDROutput\(\) \+ \(FloodLights\.GetSideOutput\(\)"),
    ("LMP flood", r"floodLight_Right->SetIntensity\(FloodLights\.GetLMPOutput\(\) \+ \(FloodLights\.GetSideOutput\(\)"),
    ("ORDEAL", r"IntegralLights_LMVC_Ordeal, MAT_EMISSION, ordeal\.LightingPower\(\)"),
    ("panels 8, 11", r"\bLtgSidePanelsSwitch\.GetState\(\) == TOGGLESWITCH_UP\)\s*\{\s*// Panel 8, 11"),
    ("panels 12, 14, 16", r"(?<![A-Za-z_])SidePanelsSwitch\.GetState\(\) == TOGGLESWITCH_UP\)\s*\{\s*// Panel 12, 14, 16"),
]


def check_lighting_sources(vc):
    """Stop unless the VC's lighting case still scales each channel the way
    LMDD_GET_LIGHTING reports it."""
    m = re.search(r"case AID_LMVC_LIGHTING:(.*?)#define XP_LIT_ON", vc, re.S)
    if not m:
        sys.exit("apply_patch: lemvc.cpp: no AID_LMVC_LIGHTING case before the legends")
    for what, pat in LIGHTING_SOURCES:
        if not re.search(pat, m.group(1)):
            sys.exit(f"apply_patch: lemvc.cpp: the VC no longer lights the {what} "
                     f"channel as LMDD_GET_LIGHTING reports it - read the "
                     f"AID_LMVC_LIGHTING case and update both")


def norm(code):
    """Whitespace-insensitive form of a code block, for the identity check."""
    return re.sub(r"\s+", "", code)


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    p = Patch(sys.argv[1])

    # 1. A connector type, appended so no existing ordinal moves.
    p.after("src_sys/connector.h", "\tVHF_RNG,\n",
            "\tLM_DISPLAY_DATA,        ///< LEMLINK LOCAL PATCH: read-only LM display data.\n",
            "ConnectorType enum")

    # 2-3. A meter lookup on PanelSwitches.
    p.after("src_sys/toggleswitch.h",
            "\tbool SetState(const char *n, int value, bool guard = false, bool hold = false);\n",
            "\tMeterSwitch *GetMeterByName(const char *n);    // LEMLINK LOCAL PATCH\n",
            "GetMeterByName declaration")
    p.after("src_sys/toggleswitch.h",
            "\tMeterSwitch *GetMeterByName(const char *n);    // LEMLINK LOCAL PATCH\n",
            "\tContinuousSwitch *GetContinuousByName(const char *n);    // LEMLINK LOCAL PATCH\n",
            "GetContinuousByName declaration")
    # A continuous knob's own position as a double. SetState(int) is the only
    # public setter and rounds to whole display units, which is what limited a
    # pit pot to 9 settings on a lighting knob. Inline and non-virtual: no
    # member, no vtable slot, so every other module's layout is untouched.
    p.after("src_sys/toggleswitch.h",
            "\t//Returns displayed value (not animation state)\n\tdouble GetValue();\n",
            "\tvoid SetDisplayValue(double value) { SwitchTo(DisplayToAngle(value)); }   // LEMLINK LOCAL PATCH\n",
            "ContinuousSwitch::SetDisplayValue")
    s = p._load("src_sys/toggleswitch.cpp")
    p.text["src_sys/toggleswitch.cpp"] = s + """
//
// LEMLINK LOCAL PATCH. The panel-wide walk SetState() does inline, returning
// the item instead of setting it. dynamic_cast because the name may belong to
// a switch rather than a meter, and 0 is the honest answer for that.
//
MeterSwitch *PanelSwitches::GetMeterByName(const char *n)

{
	SwitchRow *row = RowList;

	while (row) {
		PanelSwitchItem *p = row->GetItemByName(n);
		if (p) {
			return dynamic_cast<MeterSwitch *>(p);
		}
		row = row->GetNext();
	}

	return 0;
}

//
// LEMLINK LOCAL PATCH. The same walk for a continuous knob (the lighting and
// S-band antenna knobs, ORDEAL ALT SET); 0 for any other kind of item.
//
ContinuousSwitch *PanelSwitches::GetContinuousByName(const char *n)

{
	SwitchRow *row = RowList;

	while (row) {
		PanelSwitchItem *p = row->GetItemByName(n);
		if (p) {
			return dynamic_cast<ContinuousSwitch *>(p);
		}
		row = row->GetNext();
	}

	return 0;
}
"""

    # 4. The FDAI ball, which is protected. An inline getter adds no data
    # member and no virtual, so the layout every other module sees is intact.
    p.after("src_sys/FDAI.h", "\tbool IsPowered();\n\tvoid SetLMmode();\n",
            "\tVECTOR3 GetBallAttitude() const { return now; }   // LEMLINK LOCAL PATCH\n",
            "FDAI ball getter")

    # 5. LEM.h: the connector, its friendship, the FDAI helper, and the two
    # friendships into classes whose display state is private.
    lem_h = "src_lm/LEM.h"
    p.after(lem_h, '#include "lemconnector.h"\n',
            '#include "lemdisplaydata.h"                     // LEMLINK LOCAL PATCH\n',
            "include")
    p.after(lem_h, "\tPanelConnector MFDToPanelConnector;\n",
            "\tvoid GetFDAIRatesErrors(bool right, VECTOR3 &rates, VECTOR3 &errors); // LEMLINK LOCAL PATCH\n",
            "FDAI helper")
    # The one DATA member this patch adds goes LAST in class LEM. mcc.dll,
    # ApolloRTCCMFD, ProjectApolloMFD and LC37 are built against stock LEM.h
    # and touch LEM members by offset (MCC::LM_uplink memcpy's into
    # lm->PCM.mcc_data). A member inserted mid-class moved every later one
    # and the Apollo 5 uplink crashed in mcc.dll. Only LEM.dll
    # constructs a LEM, so growing it at the end is safe.
    p.replace(lem_h,
              "\tfriend void cbLMVesim(int inputID, int eventType, int newValue, void *pdata);\n};\n",
              "\tfriend void cbLMVesim(int inputID, int eventType, int newValue, void *pdata);\n"
              "\n\t// LEMLINK LOCAL PATCH: last, so no member another DLL reaches moves.\n"
              "\tLEMDisplayDataConnector displayDataConnector;\n};\n",
              "connector member, at the end of class LEM")
    p.after(lem_h, "\tfriend class LEM_EDS;\n",
            "\tfriend class LEMDisplayDataConnector;       // LEMLINK LOCAL PATCH\n",
            "LEM friendship")
    p.replace(lem_h, "\tbool IsPowered();\n\nprivate:\n\tvoid TapeDrive(",
              "\tbool IsPowered();\n\tfriend class LEMDisplayDataConnector;   // LEMLINK LOCAL PATCH\n"
              "\nprivate:\n\tvoid TapeDrive(",
              "LEM_RadarTape friendship")
    p.after("src_lm/lm_ags.h", "\tbool HasNumPower();\n\tbool HasIntglPower();\n",
            "\tfriend class LEMDisplayDataConnector;   // LEMLINK LOCAL PATCH\n",
            "LEM_DEDA friendship")
    # Build 47 renamed the DSKY's public power getters and changed what they
    # mean (GetDSKYPower() reads the DC input, not the EL supply the digits
    # are drawn from), so read the drawn condition itself, as a friend.
    p.replace("src_sys/dsky.h", "protected:\n\n\tbool IsStatusPowered();\n",
              "\tfriend class LEMDisplayDataConnector;   // LEMLINK LOCAL PATCH\n"
              "protected:\n\n\tbool IsStatusPowered();\n",
              "DSKY friendship")

    # 6. LEM.cpp: construct and register.
    p.after("src_lm/LEM.cpp", "\tMFDToPanelConnector(MainPanel, checkControl),\n",
            "\tdisplayDataConnector(this),                   // LEMLINK LOCAL PATCH\n",
            "connector construction")
    p.after("src_lm/LEM.cpp",
            "\tRegisterConnector(VIRTUAL_CONNECTOR_PORT, &MFDToPanelConnector);\n",
            "\tRegisterConnector(VIRTUAL_CONNECTOR_PORT, &displayDataConnector); // LEMLINK LOCAL PATCH\n",
            "connector registration")

    # 7. The FDAI helper, and its four call sites.
    body = "\n".join(("\t" + line) if line.strip() else ""
                     for line in FDAI_BODY.strip("\n").split("\n"))
    body = body.replace("RATEERR", "rateErrMon").replace("ATTMON", "attMon")
    p.replace("src_lm/lempanel.cpp", "void LEM::RedrawPanel_XPointer(",
              HELPER.replace("BODY", body + "\n").lstrip("\n")
              + "void LEM::RedrawPanel_XPointer(",
              "FDAI helper definition")
    for rel in ("src_lm/lempanel.cpp", "src_lm/lemvc.cpp"):
        p.fdai_site(rel, "RateErrorMonSwitch", "AttitudeMonSwitch",
                    "left FDAI")
        p.fdai_site(rel, "RightRateErrorMonSwitch", "RightAttitudeMonSwitch",
                    "right FDAI")

    # 7b. The self-drawn lamps, as one helper generated from the panel's own
    # conditions. Added beside the FDAI helper; it reads, never draws.
    pan = p._load("src_lm/lempanel.cpp")
    start = pan.index("bool LEM::clbkPanelRedrawEvent")
    redraw = pan[start:]
    lines = []
    for bit, aid in LAMPS:
        lines.append(f"\t// {aid}\n\tif ({lamp_condition(redraw, aid)}) lit |= {bit};")
    lamp_helper = ("//\n// LEMLINK LOCAL PATCH. Every lamp the panel draws from subsystem logic,\n"
                   "// one bit each. Generated by apply_patch.py from the cases in\n"
                   "// clbkPanelRedrawEvent below - do not edit by hand.\n//\n"
                   "unsigned LEM::GetPanelLampBits()\n{\n\tunsigned lit = 0;\n"
                   + "\n".join(lines) + "\n\treturn lit;\n}\n\n")
    p.replace("src_lm/lempanel.cpp", "void LEM::RedrawPanel_XPointer(",
              lamp_helper + "void LEM::RedrawPanel_XPointer(", "panel lamp helper")
    p.after(lem_h, "\tvoid GetFDAIRatesErrors(bool right, VECTOR3 &rates, VECTOR3 &errors); // LEMLINK LOCAL PATCH\n",
            "\tunsigned GetPanelLampBits();                 // LEMLINK LOCAL PATCH\n",
            "panel lamp helper declaration")

    # 7c. A NASSP bug fix, the first edit that changes behaviour rather than
    # reading: LMAbortButton declared
    # `bool SwitchTo(int newState)`, which HIDES the base class's virtual
    # `SwitchTo(int newState, bool dontspring = false)` instead of overriding
    # it. The mouse and VESIM paths call AbortSwitch.SwitchTo() on the derived
    # type and reach the code that sets the AGC/AEA abort discretes; the
    # connector's PanelSwitches::SetState goes through the virtual and never
    # does, so ABORT from LEMLink set the button and the LGC never aborted.
    # Matching the signature makes it an override; behaviour on the other
    # paths is unchanged.
    p.replace("src_lm/lemswitches.h",
              "\tbool SwitchTo(int newState);\nprotected:\n\tLEM *lem;\n};\n\nclass LMAbortStageButton",
              "\tbool SwitchTo(int newState, bool dontspring = false); // LEMLINK LOCAL PATCH: override, not hide\n"
              "protected:\n\tLEM *lem;\n};\n\nclass LMAbortStageButton",
              "LMAbortButton::SwitchTo declaration")
    p.replace("src_lm/lemswitches.cpp",
              "bool LMAbortButton::SwitchTo(int newState)\n{\n\tif (TwoPositionSwitch::SwitchTo(newState))",
              "bool LMAbortButton::SwitchTo(int newState, bool dontspring) // LEMLINK LOCAL PATCH\n{\n"
              "\tif (TwoPositionSwitch::SwitchTo(newState, dontspring))",
              "LMAbortButton::SwitchTo definition")

    # 7d. NASSP bug fix (build 47): CrossPointer::Init copies the relay
    # states it is given - Get9K32A() and friends return bools by value - once,
    # at SystemsInit, and nothing refreshes them. Panel1/2RelayBox update the
    # relays every timestep (the radar tape reads them live and works), but
    # both X-pointers stay on whatever source the relays showed at load: the
    # PGNS branch, so RATE/ERR MON RNDZ RADAR and MODE SEL LDG RADAR / AGS do
    # nothing and the needles read zero once the LGC stops driving them. Build
    # 27 read the switches directly. Re-read the same relays Init() was given,
    # each step, into the same members - no new data member, because LEM
    # holds two CrossPointers and other DLLs depend on LEM's layout.
    p.replace("src_lm/lemsystems.cpp",
              "void CrossPointer::Timestep(double simdt)\n{\n\tif (!IsPowered())",
              "void CrossPointer::Timestep(double simdt)\n{\n"
              "\t// LEMLINK LOCAL PATCH: refresh the relays Init() copied once at SystemsInit.\n"
              "\tbool left = (this == &lem->crossPointerLeft);\n"
              "\tRateErrorDispRelay = left ? lem->Panel1RelayBox.Get9K32A() : lem->Panel2RelayBox.Get9K30A();\n"
              "\tRateErrorLtRelay = left ? lem->Panel1RelayBox.Get9K32B() : lem->Panel2RelayBox.Get9K30B();\n"
              "\tModeSelLRDispRelay = lem->Panel2RelayBox.Get9K29B();\n"
              "\tModeSelAGSDispRelay = lem->Panel2RelayBox.Get9K34B();\n"
              "\tModeSelAGSLtRelay = lem->Panel2RelayBox.Get9K34A();\n\n"
              "\tif (!IsPowered())",
              "CrossPointer relay refresh")
    # And the display branch has the rate/error relay backwards, which the
    # frozen copies hid: 9K32A/9K30A energise with RATE/ERR MON DOWN (LDG
    # RDR/CMPTR, Panel1/2RelayBox::Timestep), and the lighting below already
    # reads them that way (`if (!RateErrorLtRelay)` lights the RR legends).
    # So the rendezvous radar is the de-energised branch.
    p.replace("src_lm/lemsystems.cpp",
              "\tif (RateErrorDispRelay) //Rendezvous Radar",
              "\tif (!RateErrorDispRelay) //Rendezvous Radar (LEMLINK LOCAL PATCH: relay energises at LDG RDR/CMPTR)",
              "CrossPointer rate/error polarity")

    # 7e. Altitude-rate tape direction: upstream commit
    # 6d7d2dfb67 ("Correct range rate scale (black negative)", build 47)
    # flipped the drum's sign with no change to the tape art, so a descent
    # runs onto the black half and reads as a climb; after touchdown the held
    # LR rate (-3 ft/s) shows +3. NASSP's own init line still places -240 fps
    # at 3759 "per LM closeout images", which is the old direction. And the
    # LM-11 operations handbook vol 2 para 4.8.2.2 settles it: "If range rate
    # value is displayed on black portion of RANGE RATE ind tape, positive
    # range rate DEDA entry is indicated; negative is closing rate."
    p.replace("src_lm/lemsystems.cpp",
              "\tdesRate = 2881.0 - 82.0 + (reqRate * 3.2808399 * 40.0 * 100.0 / 1000.0);",
              "\tdesRate = 2881.0 - 82.0 - (reqRate * 3.2808399 * 40.0 * 100.0 / 1000.0); // LEMLINK LOCAL PATCH: the direction before upstream 6d7d2dfb67",
              "radar tape rate direction")

    # 7f. Friendship in EngineStartButton, so the connector can report its
    # lamp through the button's own protected LightLogic(). No layout change.
    p.replace("src_lm/lemswitches.h",
              "class EngineStartButton : public PushSwitch {\n\npublic:\n",
              "class EngineStartButton : public PushSwitch {\n"
              "\tfriend class LEMDisplayDataConnector;   // LEMLINK LOCAL PATCH\n\npublic:\n",
              "EngineStartButton friendship")

    # 7g. The VC-only legends: X-pointer source and
    # scale, the tape's ALT / RANGE, RCS He x10. The 2D panel draws none of
    # them; the VC is the source of truth, so its own block is copied into a
    # reader appended to lemvc.cpp, where the VC_MAT_ ids are in scope.
    vc = p._load("src_lm/lemvc.cpp")
    check_lighting_sources(vc)          # 7h: what LMDD_GET_LIGHTING mirrors
    p.text["src_lm/lemvc.cpp"] = vc + legend_helper(vc_legend_block(vc))
    p.after(lem_h, "\tunsigned GetPanelLampBits();                 // LEMLINK LOCAL PATCH\n",
            "\tunsigned GetVCLegendBits();                  // LEMLINK LOCAL PATCH\n",
            "VC legend helper declaration")

    # 7i. Talkbacks read as drawn. IndicatorSwitch::
    # DrawSwitch checks the item's breaker (SRC) itself and draws
    # `failOpen ? 1 : 0` when it is dead, so a GetState() that skips the check
    # still draws right but answers the connector wrongly. The SCEA, double
    # SCEA and DPS-valve talkbacks repeat the check inside GetState(); these
    # three do not. Nothing in NASSP but the two draw paths and the connector
    # calls their GetState(), so drawing is unchanged.
    p.replace("src_lm/lemswitches.cpp",
              "int LEMRCSQuadTalkback::GetState()\n{\n\tif (ssswitch && tcaFailure)",
              "int LEMRCSQuadTalkback::GetState()\n{\n"
              "\tif (SRC && SRC->Voltage() <= SP_MIN_DCVOLTAGE)   // LEMLINK LOCAL PATCH: as drawn\n"
              "\t\treturn (failOpen ? 1 : 0);\n\n"
              "\tif (ssswitch && tcaFailure)",
              "RCS quad talkback power check")
    p.replace("src_lm/lemswitches.cpp",
              "int RecorderTalkback::GetState()\n{\n\treturn dsea->TapeMotion() ? 1 : 0;",
              "int RecorderTalkback::GetState()\n{\n"
              "\tif (SRC && SRC->Voltage() <= SP_MIN_DCVOLTAGE)   // LEMLINK LOCAL PATCH: as drawn\n"
              "\t\treturn (failOpen ? 1 : 0);\n\n"
              "\treturn dsea->TapeMotion() ? 1 : 0;",
              "recorder talkback power check")
    # DES BATS is a plain IndicatorSwitch whose state lm_eps.cpp sets, and its
    # GetState() is the inline base one in toggleswitch.h, shared with the
    # CSM. So gate the state where it is set, on the breaker it is wired to
    # (EPS_DISP_CB, lemsystems.cpp); unpowered it draws 0 either way.
    p.replace("src_lm/lm_eps.cpp",
              "\t//DES BATTS TB\n\tif (DFR && lem->drb.GetDeadFaceRelay())",
              "\t//DES BATTS TB\n"
              "\tif (DFR && lem->drb.GetDeadFaceRelay() && lem->EPS_DISP_CB.Voltage() > SP_MIN_DCVOLTAGE) // LEMLINK LOCAL PATCH: as drawn",
              "DES BATS talkback power check")

    # 7j. RR SLEW diagonals. A panel item holds one state and each state moves
    # one axis, and alternating through S clicks on every swap at half the rate.
    # So the second contact of a diagonal is a second item, which LEM_RR reads
    # with the same position numbers. It is in RadarSlewSwitchRow, whose draw and
    # mouse loops call it and get nothing. PanelSwitchItem::SetState plays no
    # sound, and it never registers with PSH, so no scenario carries it.
    p.replace("src_lm/lemswitches.h", "class LMAbortStageButton : public GuardedPushSwitch {",
              "// LEMLINK LOCAL PATCH: the second contact of RR SLEW, a name and a state only.\n"
              "class RadarSlewContact2 : public PanelSwitchItem {\n"
              "public:\n"
              "\tRadarSlewContact2() { name = (char *)\"RadarSlewSwitch2\"; state = FIVEPOSSWITCH_CENTER; }\n"
              "\tbool CheckMouseClick(int event, int mx, int my) { return false; }\n"
              "\tvoid DrawSwitch(SURFHANDLE DrawSurface) {}\n"
              "\tvoid SaveState(FILEHANDLE scn) {}\n"
              "\tvoid LoadState(char *line) {}\n"
              "};\n\n"
              "class LMAbortStageButton : public GuardedPushSwitch {",
              "RadarSlewContact2 class")
    p.after(lem_h, "\tLEMDisplayDataConnector displayDataConnector;\n",
            "\tRadarSlewContact2 RadarSlewSwitch2;\n",
            "RadarSlewSwitch2 member, after the connector at the end of class LEM")
    p.after("src_lm/lempanel.cpp",
            "\tRadarSlewSwitch.Init(0, 0, 39, 39, srf[SRF_FIVE_POS_SWITCH], NULL, RadarSlewSwitchRow);\n",
            "\tRadarSlewSwitchRow.AddSwitch(&RadarSlewSwitch2);   // LEMLINK LOCAL PATCH\n",
            "RadarSlewSwitch2 in its row")
    for pos in (4, 3, 2, 0):
        test = f"lem->RadarSlewSwitch.GetState() == {pos}"
        p.replace("src_lm/lm_rr.cpp", f"if ({test})",
                  f"if ({test} || lem->RadarSlewSwitch2.GetState() == {pos})",
                  f"RR slew position {pos}")

    # 8. Build the new source.
    p.after("Project/LEM/CMakeLists.txt",
            "    ${NASSP_SOURCE_ROOT}/src_lm/lemconnector.cpp\n",
            "    ${NASSP_SOURCE_ROOT}/src_lm/lemdisplaydata.cpp\n",
            "CMake source list")

    p.write()
    print(f"apply_patch: {len(p.text)} files edited, "
          f"{len(NEW_FILES)} added under src_lm/")


if __name__ == "__main__":
    main()
