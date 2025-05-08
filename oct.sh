#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 2 ]; then
  echo "Usage: $0 <case-name> <input.ses>"
  echo "Available cases: 59io 99io 256io 300io 400io 500io"
  exit 1
fi

CASE="$1"
SES_FILE="$2"
BASE="${SES_FILE%.*}"

# 先定義中間檔名
IO_INFO="${BASE}_io_info.txt"
SEGMENTS="${BASE}_segments.txt"
VORONOI_OUT="${BASE}_voronoi_result.txt"
INNER_OUT="eliminated_inner_boundary_result.txt"
OUTER_OUT="ExtendBoundaryToOutterRect_result.txt"
OCT_OUT="ooo_nets_info.txt"

# 根據 case 給不同參數
case "$CASE" in
  59io)
    ELIM_PARAMS=( -5  -5  10   10 )
    EXT_PARAMS=( -1.4 -1.4 2.8  2.8 )
    ;;
  99io)
    ELIM_PARAMS=( -10 -10 20   20 )
    EXT_PARAMS=( -2.0 -2.0 4.0  4.0 )
    ;;
  256io)
    ELIM_PARAMS=(-5e+07 -5e+07 1e+08 1e+08)
    EXT_PARAMS=(-1.4e+08 -1.4e+08 2.8e+08 2.8e+08)
    ;;
	300io)
	ELIM_PARAMS=( -5e+07  -5e+07  1e+08   1e+08 )
    EXT_PARAMS=( -1.4e+08 -1.4e+08 2.8e+08  2.8e+08 )
    ;;
	400io)
	ELIM_PARAMS=( -5e+07  -5e+07  1e+08   1e+08 )
    EXT_PARAMS=( -1.4e+08 -1.4e+08 2.8e+08  2.8e+08 )
    ;;
	500io)
	ELIM_PARAMS=( -5e+07  -5e+07  1e+08   1e+08 )
    EXT_PARAMS=( -1.4e+08 -1.4e+08 2.8e+08  2.8e+08 )
    ;;
  *)
    echo "Unknown case: $CASE"
    exit 1
    ;;
esac

echo "Running case $CASE on $SES_FILE..."
echo "1) parse & segment: $SES_FILE → $IO_INFO, $SEGMENTS"
./parse_ses_all   "$SES_FILE" "$IO_INFO" "$SEGMENTS"
echo "2) voronoi: $SEGMENTS → ${SEGMENTS%.txt}_voronoi_result.txt"
./basic_backup "$SEGMENTS" > "$VORONOI_OUT"
echo "3) eliminate inner boundary → eliminated_inner_boundary_result.txt"
./eliminated_inner_boundary   "$VORONOI_OUT"   "${ELIM_PARAMS[@]}"
echo "4) extend to outer rect → ExtendBoundaryToOutterRect_result.txt"
./ExtendBoundaryToOutterRect  "$INNER_OUT"      "${EXT_PARAMS[@]}"
echo "5) octlinearize with both param sets → ooo_nets_info.txt"
./OctilinearizeBoundary    "$OUTER_OUT"      
   # \"${ELIM_PARAMS[@]}" "${EXT_PARAMS[@]}"

echo "All done!"
