#!/bin/bash
# generate_cyclonedds_types.sh
# Generate CycloneDDS C type files from IDL definitions
#
# Prerequisites: idlc must be in PATH (installed with CycloneDDS)
#
# Usage: ./generate_cyclonedds_types.sh [output_dir]
#
# If output_dir is not specified, defaults to ../dds/cyclonedds/types
# relative to the script location.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IDL_DIR="${SCRIPT_DIR}/../types"
OUTPUT_DIR="${1:-${SCRIPT_DIR}/../dds/cyclonedds/types}"

# Verify idlc is available
if ! command -v idlc &> /dev/null; then
    echo "Error: idlc not found in PATH." >&2
    echo "Install CycloneDDS and ensure idlc is in your PATH." >&2
    echo "  e.g., export PATH=\$CYCLONEDDS_INSTALL/bin:\$PATH" >&2
    exit 1
fi

# Verify IDL directory exists
if [ ! -d "$IDL_DIR" ]; then
    echo "Error: IDL directory not found: $IDL_DIR" >&2
    exit 1
fi

mkdir -p "$OUTPUT_DIR"

echo "Generating CycloneDDS types from IDL files..."
echo "  IDL directory:    $IDL_DIR"
echo "  Output directory: $OUTPUT_DIR"
echo ""

# Count IDL files
IDL_COUNT=0
FAIL_COUNT=0

# Find all .idl files and generate C types
# Sort to ensure deterministic ordering
find "$IDL_DIR" -name "*.idl" | sort | while read -r idl_file; do
    rel_path="${idl_file#$IDL_DIR/}"
    echo "  Processing: $rel_path"

    if idlc -f case-sensitive -l c "$idl_file" -o "$OUTPUT_DIR" -I "$IDL_DIR"; then
        IDL_COUNT=$((IDL_COUNT + 1))
    else
        echo "    WARNING: Failed to process $rel_path" >&2
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
done

echo ""
echo "Done. Generated types in: $OUTPUT_DIR"

if [ "$FAIL_COUNT" -gt 0 ]; then
    echo "WARNING: $FAIL_COUNT file(s) failed to process." >&2
    exit 1
fi
