"""The application source as ONE text, for the host tests that extract code by pattern.
The firmware lives in Core/Inc/app/*.h + Core/Src/app/*.c; the tests were written against the old single main.c.
source() joins app_config.h and the module .c files in reading order, puts each macro that moved to a module header
back where it was, and gives 'static' back to the file-scope definitions that only lost it because another module uses them."""
import pathlib, re
ROOT = pathlib.Path(__file__).resolve().parents[1]
INC, SRC = ROOT/'Core/Inc/app', ROOT/'Core/Src/app'
MODULES = ['osc_state', 'osc_process', 'osc_settings', 'osc_timebase', 'osc_actions',
           'display', 'buttons', 'adc_capture', 'slow_mode', 'ext_points', 'osc_task', 'input_task', 'app_main']
JOINED = MODULES[:-1]       # app_main.c is not joined: its AppInit()/AppStart() bodies are inlined into main() (see _main)
# already global (no 'static') before the split: SWD-readable debug variables, the SysTick hook, the shared results
PUBLIC = {'OscSlowTick', 'new_frame_ready', 'osc_meas', 'raw_samples', 'adc_raw_buf', 'prof_us', 'AppInit', 'AppStart',
          'dbg_adc_frames', 'dbg_adc_max', 'dbg_adc_min', 'dbg_adc_ok', 'dbg_adc_pre', 'dbg_auto_fsel', 'dbg_b_burst_ms',
          'dbg_b_px_ms', 'dbg_b_spi_ms', 'dbg_fm_cnt', 'dbg_fm_fail', 'dbg_fps_x10', 'dbg_frames_total', 'dbg_hdr_ms',
          'dbg_hold_cnt', 'dbg_lcd_bar', 'dbg_lcd_set', 'dbg_loop_ms', 'dbg_pipe_cnt', 'dbg_px_frame', 'dbg_px_last',
          'dbg_sig_hash', 'dbg_t0_ms', 'dbg_uptime_ms', 'dbg_wf_ms'}

def _header_names():
    names = set()
    for m in MODULES:
        for ln in (INC/(m + '.h')).read_text().split('\n'):
            if ln.startswith('extern '):
                names.update(re.findall(r'(\w+)\s*(?:\[[^\]]*\])*\s*(?:,|;)', ln))
            else:
                f = re.match(r'^[\w\s\*]+?\b(\w+)\s*\(', ln)
                if f and not ln.startswith('#'): names.add(f.group(1))
    return names

def _define(name, text):
    out, cont = [], False
    for ln in text.split('\n'):
        if cont or re.match(r'#define\s+' + name + r'\b', ln):
            out.append(ln); cont = ln.rstrip().endswith('\\')
    assert out, name
    return out

def source():
    names = _header_names() - PUBLIC
    parts = [(INC/'app_config.h').read_text()]
    for m in JOINED:
        out = []
        for ln in (SRC/(m + '.c')).read_text().split('\n'):
            mv = re.match(r'^/\* (\w+): defined in (\w+\.h) \(used by other modules\) \*/$', ln)
            if mv:                       # a macro that moved to the module header: put it back where it was
                out += _define(mv.group(1), (INC/mv.group(2)).read_text()); continue
            if re.match(r'^[A-Za-z_]', ln) and not ln.startswith(('static ', 'extern ', 'typedef ', '#')):
                f = re.match(r'^(?:ALIGN_32BYTES\()?[\w\s\*]*?\b(\w+)\s*[\(\[=;,]', ln)
                if f and f.group(1) in names:
                    ln = 'static ' + ln
            out.append(ln)
        parts.append('\n'.join(out))
    parts.append(_main())
    return '\n'.join(parts)

def _fn(text, head):
    a = text.index(head + '\n{\n'); b = text.index('\n}\n', a)
    return a, b + 3, text[a + len(head) + 3:b + 1]          # start, end, body lines without the braces

def _main():
    """main() of Core/Src/main.c with the AppInit(); / AppStart(); calls replaced by their bodies (what runs at boot)"""
    mc = (ROOT/'Core/Src/main.c').read_text(); am = (SRC/'app_main.c').read_text()
    a, b, _ = _fn(mc, 'int main(void)'); m = mc[a:b]
    for name in ('AppInit', 'AppStart'):
        call = re.search(r'^ *%s\(\);[^\n]*\n' % name, m, re.M)
        m = m[:call.start()] + _fn(am, 'void %s(void)' % name)[2] + m[call.end():]
    return m

if __name__ == '__main__':
    print(source())
