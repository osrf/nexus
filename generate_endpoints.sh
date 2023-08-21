#!/bin/bash
set -e

here="$(dirname "$0")"
redf -g=cpp -o="$here/nexus_endpoints" "$here/nexus_endpoints.redf.yaml"
