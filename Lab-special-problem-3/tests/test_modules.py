"""Structural contract: application modules must be real translation units."""
from pathlib import Path
import re
root = Path(__file__).resolve().parents[1]
for name in ('app', 'acquisition', 'signal', 'display', 'input', 'simulator'):
    assert (root / f'Core/Src/osc_{name}.c').is_file(), f'missing module: {name}'
    assert (root / f'Core/Inc/osc_{name}.h').is_file(), f'missing interface: {name}'
for source in (root / 'Core/Src').glob('*.c'):
    assert not re.search(r'#\s*include\s*["<].*\.c[">]', source.read_text()), source
main = (root / 'Core/Src/main.c').read_text()
assert 'OscAppInit();' in main and 'OscAppStep();' in main
assert 'AdcCaptureRaw' not in main and 'DrawWaveform' not in main
print('PASS 6 separate module/interface pairs; thin main; no .c includes')
