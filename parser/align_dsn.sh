#!/usr/bin/env bash
set -euo pipefail

# align_dsn.sh
# Usage: ./align_dsn.sh <input.dsn>
# All parameters (output file, outline, pitch) are predefined per input file via case statement.

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <input.dsn>"
  exit 1
fi

INPUT="$1"
BASENAME=$(basename "$INPUT")
#ballOutline往外推線性參數(避免無法繞線)
#boundaryExpand=50
boundaryExpand=150
#ballOutline內縮比例參數
#marginPercent=5
marginPercent=5
#pad Pitch
#PITCH=26
#alpha 
alpha=0
# Predefined settings for each known input file
case "$BASENAME" in
  0507_59io_raw.dsn)
    OUTPUT="aligned_59io.dsn"
    X0=0
    Y0=0
    W=3210
    H=2100
    PITCH=55
    BALLPITCH=500
    ;;
  0507_99io_raw.dsn)
    OUTPUT="aligned_99io.dsn"
    X0=-2466
    Y0=-2488
    W=4932
    H=4976
    PITCH=66
    BALLPITCH=650
    ;;
  0507_256io_raw.dsn)
    OUTPUT="aligned_256io.dsn"
    X0=-3500
    Y0=-3500
    W=7000
    H=7000
    PITCH=80
    BALLPITCH=500
    ;;
  0507_300io_raw.dsn)
    OUTPUT="aligned_300io.dsn"
    X0=-3580
    Y0=-3580
    W=7160
    H=7160
    PITCH=50
    BALLPITCH=500
    ;;
  0507_400io_raw.dsn)
    OUTPUT="aligned_400io.dsn"
    X0=0
    Y0=0
    W=4122
    H=4230
    PITCH=26
    BALLPITCH=650
    ;;
  0507_500io_raw.dsn)
    OUTPUT="aligned_500io.dsn"
    X0=-3735
    Y0=-3735
    W=7470
    H=7470
    PITCH=30
    BALLPITCH=800
    ;;
  *)
    echo "Error: Unrecognized input file '$BASENAME'." >&2
    exit 1
    ;;
esac
# Inform user
echo "Aligning DIE1 pads for '$INPUT' -> '$OUTPUT'"
echo "  Outline: origin=(${X0},${Y0}), size=${W}x${H}, pitch=${PITCH}"

# Call the alignment tool (parse_dsn must be in PATH or same dir)
./parse_dsn "$INPUT" "$OUTPUT" "$X0" "$Y0" "$W" "$H" "$PITCH" "$BALLPITCH" "$marginPercent" "$boundaryExpand"

if [ $? -eq 0 ]; then
  echo "Success: Output written to '$OUTPUT'"
else
  echo "Failure: parse_dsn returned an error." >&2
  exit 1
fi
