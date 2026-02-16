#!/bin/bash

# Create wrapper scripts
mkdir -p install_temp

cat > install_temp/motion_server << 'EOF'
#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/suryavadhani/sfws/install/auv_control/lib/python3.10/site-packages')
from auv_control.motion_action_server import main
if __name__ == '__main__':
    main()
EOF

cat > install_temp/motion_client << 'EOF'
#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/suryavadhani/sfws/install/auv_control/lib/python3.10/site-packages')
from auv_control.motion_action_client import main
if __name__ == '__main__':
    main()
EOF

chmod +x install_temp/motion_server
chmod +x install_temp/motion_client
