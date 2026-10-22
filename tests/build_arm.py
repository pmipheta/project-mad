"""Portable ARM verification; leaves CubeIDE Debug artifacts untouched."""
from pathlib import Path
import subprocess, concurrent.futures
root=Path(__file__).resolve().parents[1]
out=Path.home()/'.hermes/cache/scratch/osc_arm_build';out.mkdir(parents=True,exist_ok=True)
tools=Path('/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.macosaarch64_1.0.0.202602081740/tools/bin')
gcc=str(tools/'arm-none-eabi-gcc')
flags=['-mcpu=cortex-m7','-mthumb','-mfpu=fpv5-d16','-mfloat-abi=hard','-std=gnu11','-DDEBUG','-DUSE_HAL_DRIVER','-DSTM32F767xx','-O0','-g3','-ffunction-sections','-fdata-sections','-Wall','-Wextra','--specs=nano.specs']
incs=['Core/Inc','Core/Inc/ILI9341','Core/Src/ILI9341','Drivers/STM32F7xx_HAL_Driver/Inc','Drivers/STM32F7xx_HAL_Driver/Inc/Legacy','Drivers/CMSIS/Device/ST/STM32F7xx/Include','Drivers/CMSIS/Include','Middlewares/FreeRTOS/Source/include','Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1']
flags += ['-I'+str(root/i) for i in incs]
sources=(sorted((root/'Core/Src').rglob('*.c'))
         +sorted((root/'Drivers/STM32F7xx_HAL_Driver/Src').glob('*.c'))
         +sorted((root/'Middlewares/FreeRTOS/Source').glob('*.c'))
         +[root/'Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c',
           root/'Middlewares/FreeRTOS/Source/portable/MemMang/heap_4.c']
         +sorted((root/'Core/Startup').glob('*.s')))
def compile(src):
 obj=out/(str(src.relative_to(root)).replace('/','_')+'.o')
 p=subprocess.run([gcc,*flags,'-c',str(src),'-o',str(obj)],capture_output=True,text=True)
 return obj,p
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:results=list(pool.map(compile,sources))
log=''.join(p.stdout+p.stderr for _,p in results)
(out/'build.log').write_text(log)
if any(p.returncode for _,p in results):print(log);raise SystemExit(1)
elf=out/'oscilloscope.elf'
cmd=[gcc,*flags,*[str(o) for o,_ in results],'-T'+str(root/'STM32F767ZITX_FLASH.ld'),'--specs=nosys.specs','-Wl,--gc-sections','-Wl,-Map='+str(out/'oscilloscope.map'),'-u','_printf_float','-Wl,--start-group','-lc','-lm','-Wl,--end-group','-o',str(elf)]
p=subprocess.run(cmd,capture_output=True,text=True);log+=p.stdout+p.stderr;(out/'build.log').write_text(log)
print(p.stdout+p.stderr)
if p.returncode:raise SystemExit(p.returncode)
print('ARM BUILD PASS:',len(sources),'objects',elf)
subprocess.run([str(tools/'arm-none-eabi-size'),str(elf)],check=True)
print('Warnings:',log.count('warning:'),'log:',out/'build.log')
