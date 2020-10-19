#! /bin/sh
#
# orca.sh
# Copyright (C) 2020 zeroos <zeroos@mX270>
#
# Distributed under terms of the MIT license.
#

#!/usr/bin/sh
xvfb-run -a /root/node_modules/.bin/orca --no-sandbox $@
