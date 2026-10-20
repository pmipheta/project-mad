"""Run every host suite and judge each one by its exit code AND its printed result
(some older scripts print FAIL lines but still exit 0). Usage: python3 run_all.py"""
import pathlib, re, subprocess, sys
HERE = pathlib.Path(__file__).resolve().parent
SUITES = ['test_autotb.py', 'test_dma.py', 'test_draw.py', 'test_hold.py', 'test_osc.py', 'test_slow.py',
          'test_buttons.py', 'test_realpic.py', 'test_zoom.py', 'test_blink.py', 'test_eightbtn.py',
          'test_rtos_step3a.py', 'test_vdiv_range.py', 'test_edge_filter.py']
# test_rtos_step1.py is superseded by test_rtos_step3a.py (step 1 asserted "no mutex yet").
bad = 0
for s in SUITES:
    r = subprocess.run([sys.executable, str(HERE/s)], cwd=HERE, capture_output=True, text=True)
    out = r.stdout + r.stderr
    fails = [l for l in out.splitlines() if l.startswith('FAIL')]
    counts = [int(n) for n in re.findall(r'failures=(\d+)', out)]
    ok = r.returncode == 0 and not fails and counts and all(n == 0 for n in counts)
    print(('PASS ' if ok else 'FAIL ') + s)
    if not ok:
        bad += 1
        for l in (fails or out.strip().splitlines()[-3:])[:6]: print('    ' + l)
print('suites failed: %d of %d' % (bad, len(SUITES)))
sys.exit(1 if bad else 0)
