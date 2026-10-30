"""Check the firmware's source inventory and CubeMX configuration."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "cubemx_config"
project = ET.parse(FW / "MDK-ARM/cubemx_config.uvprojx")
sources = []
for node in project.findall(".//Groups/Group/Files/File"):
    source = (FW / "MDK-ARM" / node.findtext("FilePath").replace("\\", "/")).resolve()
    assert source.is_file(), f"Missing source: {source}"
    sources.append(source)
assert any(p.name == "app_rtos.c" for p in sources)
assert any(p.name == "port.c" for p in sources)
assert sum(p.name.startswith("heap_") for p in sources) == 1
ioc = (FW / "cubemx_config.ioc").read_text(encoding="utf-8")
keys = [line.split("=", 1)[0] for line in ioc.splitlines() if "=" in line]
assert len(keys) == len(set(keys)), "Duplicate CubeMX keys"
for folder in [FW / "Core", FW / "Middlewares"]:
    for path in folder.rglob("*"):
        if path.suffix in {".c", ".h"}:
            text = path.read_text(encoding="utf-8", errors="replace")
            assert not re.search(r"^(<<<<<<< |=======\s*$|>>>>>>> )", text, re.M), path
assert not re.search(r"^(<<<<<<< |=======\s*$|>>>>>>> )", ioc, re.M)
values = dict(line.split("=", 1) for line in ioc.splitlines() if "=" in line)
for label, count in [("IP", "IPNb"), ("Pin", "PinsNb")]:
    indexed = [k for k in values if re.fullmatch(r"Mcu\." + label + r"\d+", k)]
    assert len(indexed) == int(values["Mcu." + count]), f"Invalid {count}"
print(f"OK: {len(sources)} project entries; RTOS sources and CubeMX configuration are consistent.")
