import os, sys
pkg_dir = os.path.dirname(os.path.abspath(__file__))        # .../<your_pkg>/scripts
pkg_dir = os.path.dirname(pkg_dir)                          # .../<your_pkg>
vendor_dir = os.path.join(pkg_dir, "vendor")                # .../<your_pkg>/vendor
if vendor_dir not in sys.path:
    sys.path.insert(0, vendor_dir)

import rosa  # now works if vendor/rosa contains the rosa python package