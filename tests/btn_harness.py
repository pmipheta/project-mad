"""v77 (RTOS step 3b) host-harness helper: extracts the REAL ButtonSample / BtnEmit / BtnDispatchEv / HandleButtons from main.c text and supplies a fake
FreeRTOS queue so a harness can compile sampler+dispatch and call HandleButtons(now) exactly as vMainTask does (btn_task_ok = 0 -> HandleButtons runs the
sampler itself, then drains the queue and dispatches). For a pre-3b source (combined HandleButtons) split_names() is empty and nothing extra is emitted."""
import re

def strip_comments(t):
    return re.sub(r'/\*.*?\*/', '', t, flags=re.S)

def func_text(nocom, name):
    m = re.search(r'^(?:static\s+)?(?:void|int|float|uint8_t|const char\s*\*)\s*' + name + r'\([^;]*?\)\s*\{', nocom, re.M)
    assert m, name
    p = nocom.index('{', m.start()); d = 1; i = p + 1
    while d:
        d += (nocom[i] == '{') - (nocom[i] == '}'); i += 1
    return nocom[m.start():i]

def is_split(raw):
    return 'static void ButtonSample(' in raw

def split_names(raw):
    return ['BtnEmit', 'ButtonSample', 'BtnDispatchEv'] if is_split(raw) else []

def prelude(raw):
    """C text placed before the extracted functions (empty for a pre-3b source)."""
    if not is_split(raw): return ''
    nc = strip_comments(raw)
    enum = re.search(r'enum \{ BE_UP[^;]*;', nc)[0]
    typedef = re.search(r'typedef struct \{ uint8_t id; uint8_t arg; \} BtnEv;', nc)[0]
    return enum + '\n' + typedef + r'''
typedef int *QueueHandle_t; typedef int BaseType_t;
#define pdPASS 1
static int fq_dummy; static QueueHandle_t xBtnQ = &fq_dummy; static uint8_t btn_task_ok = 0; volatile uint32_t dbg_btn_q_drop;
static BtnEv fq_buf[16]; static int fq_h = 0, fq_n = 0;
static BaseType_t xQueueSend(QueueHandle_t q, const void *e, int to){ (void)q;(void)to; if (fq_n >= 16) return 0; fq_buf[(fq_h+fq_n++)%16] = *(const BtnEv*)e; return pdPASS; }
static BaseType_t xQueueReceive(QueueHandle_t q, void *e, int to){ (void)q;(void)to; if (!fq_n) return 0; *(BtnEv*)e = fq_buf[fq_h]; fq_h = (fq_h+1)%16; fq_n--; return pdPASS; }
static void BtnDispatchEv(const BtnEv *e);
'''
