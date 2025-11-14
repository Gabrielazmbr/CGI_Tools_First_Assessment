import importlib
import sys

proj_path = "/home/s5819176/Desktop/MAYA-CODING/CGI_Tools_First_Assessment"
if proj_path not in sys.path:
    sys.path.append(proj_path)

modules_to_reload = ["Classes.Ball", "Classes.Mobius_Stair", "utils.maya_helpers", "main"]
for mod_name in modules_to_reload:
    if mod_name in sys.modules:
        importlib.reload(sys.modules[mod_name])
    else:
        globals()[mod_name.split(".")[-1]] = importlib.import_module(mod_name)

import main

importlib.reload(main)
main.main()
