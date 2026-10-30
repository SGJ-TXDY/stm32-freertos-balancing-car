"""Build the STM32F103C8 firmware with GNU Arm Embedded and Python 3."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "cubemx_config"
RTOS = FW / "Middlewares/Third_Party/FreeRTOS"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", type=Path, help="Directory containing arm-none-eabi-gcc")
    args = parser.parse_args()
    binary = "arm-none-eabi-gcc" + (".exe" if os.name == "nt" else "")
    gcc = str(args.toolchain / binary) if args.toolchain else shutil.which(binary)
    if not gcc or not Path(gcc).is_file():
        parser.error("GNU Arm Embedded compiler not found; use --toolchain <bin-directory>")
    prefix = gcc.removesuffix("gcc.exe").removesuffix("gcc")
    suffix = ".exe" if os.name == "nt" else ""
    out = ROOT / "build"
    out.mkdir(exist_ok=True)
    includes = [FW / "Core/Inc", FW / "Drivers/STM32F1xx_HAL_Driver/Inc",
                FW / "Drivers/STM32F1xx_HAL_Driver/Inc/Legacy",
                FW / "Drivers/CMSIS/Include",
                FW / "Drivers/CMSIS/Device/ST/STM32F1xx/Include",
                RTOS / "include", RTOS / "portable/GCC/ARM_CM3"]
    common = ["-mcpu=cortex-m3", "-mthumb", "-mfloat-abi=soft", "-Os", "-g3",
              "-ffunction-sections", "-fdata-sections", "-DUSE_HAL_DRIVER", "-DSTM32F103xB"]
    flags = common + ["-std=gnu11", "-Wall", "-Wextra", "-Wno-unused-parameter",
                      "-Werror=implicit-function-declaration"]
    for directory in includes:
        flags += ["-I", str(directory)]
    project = ET.parse(FW / "MDK-ARM/cubemx_config.uvprojx")
    sources = []
    for node in project.findall(".//Groups/Group/Files/File"):
        path = (FW / "MDK-ARM" / node.findtext("FilePath").replace("\\", "/")).resolve()
        if path.suffix == ".c":
            sources.append(path)
    sources += [FW / "GCC/syscalls.c",
                FW / "Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s"]
    objects = []
    for i, source in enumerate(dict.fromkeys(sources)):
        obj = out / f"{i:02d}_{source.stem}.o"
        print(f"CC {source.relative_to(ROOT)}", flush=True)
        subprocess.run([gcc, *flags, "-c", str(source), "-o", str(obj)], check=True)
        objects.append(str(obj))
    elf = out / "balancing-car.elf"
    subprocess.run([gcc, *common, *objects, "--specs=nano.specs", "--specs=nosys.specs",
                    "-T", str(FW / "GCC/STM32F103C8Tx_FLASH.ld"), "-Wl,--gc-sections",
                    "-Wl,--print-memory-usage", f"-Wl,-Map={out / 'balancing-car.map'}",
                    "-u", "_printf_float", "-lm", "-o", str(elf)], check=True)
    for fmt, ext in [("ihex", "hex"), ("binary", "bin")]:
        subprocess.run([prefix + "objcopy" + suffix, "-O", fmt, str(elf),
                        str(out / f"balancing-car.{ext}")], check=True)
    subprocess.run([prefix + "size" + suffix, str(elf)], check=True)
    print(f"Firmware: {elf}")


if __name__ == "__main__":
    main()
