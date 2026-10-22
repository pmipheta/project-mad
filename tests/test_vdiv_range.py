"""V/DIV selectable range contract: 100 uV/div through 2 V/div, 1-2-5 sequence."""
import re, pathlib, sys
root=pathlib.Path(__file__).resolve().parents[1]
s=(root/'Core/Src/main.c').read_text()
fails=0
def check(name, ok):
 global fails
 print(('PASS ' if ok else 'FAIL ')+name); fails += not ok
m=re.search(r'static const float\s+osc_vdivs\[\]\s*=\s*\{([^}]*)\}',s,re.S)
check('osc_vdivs table exists', bool(m))
vals=[]
if m:
 vals=[float(x.rstrip('fF')) for x in re.findall(r'(?:\d+\.\d+|\d+)[fF]?',m.group(1))]
check('minimum V/DIV is exactly 0.0001 V', bool(vals) and abs(vals[0]-0.0001)<1e-9)
check('full 1-2-5 range through 2 V', vals == [0.0001,0.0002,0.0005,0.001,0.002,0.005,0.01,0.02,0.05,0.1,0.2,0.5,1.0,2.0])
check('power-on remains 1.0 V/div', 'static int      osc_vdiv_i    = OSC_VDIV_DEF;' in s and '#define OSC_VDIV_DEF      12' in s)
check('manual decrement can reach table index zero', 'OscClampI(osc_vdiv_i + dir, 0,' in s)
check('four-decimal display preserves 0.0001', '"V:%.4fV"' in s and '"V:%.4f"' in s)
print('vdiv_range failures=%d'%fails)
sys.exit(1 if fails else 0)
