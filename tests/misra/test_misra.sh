#!/bin/bash -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PANDA_DIR="$SCRIPT_DIR/../.."
SUPPRESSIONS_LIST="$SCRIPT_DIR/suppressions.txt"
if [[ -z "${MISRA_ADDON:-}" ]]; then
  if [[ -f /usr/share/cppcheck/addons/misra.py ]]; then
    MISRA_ADDON=/usr/share/cppcheck/addons/misra.py
  else
    MISRA_ADDON=/usr/lib/x86_64-linux-gnu/cppcheck/addons/misra.py
  fi
fi
MISRA_SUPPRESS_RULES="${MISRA_SUPPRESS_RULES:-$(
  grep -oE 'misra-c2012-[0-9]+\.[0-9]+' "$SUPPRESSIONS_LIST" 2>/dev/null | sed 's/misra-c2012-//' | paste -sd, - || true
)}"
MISRA_SUPPRESS_ARGS=()
if [[ -n "$MISRA_SUPPRESS_RULES" ]]; then
  MISRA_SUPPRESS_ARGS=(--suppress-rules "$MISRA_SUPPRESS_RULES")
fi

if [[ -z "${MISRA_OUTPUT_DIR:-}" ]]; then
  if [[ -n "${GITHUB_ACTIONS:-}" ]]; then
    MISRA_OUTPUT_DIR="$PANDA_DIR/.misra"
  else
    MISRA_OUTPUT_DIR="/tmp/misra"
  fi
fi

mkdir -p "$MISRA_OUTPUT_DIR"
ERROR_CODE=0

# generate coverage matrix
#python tests/misra/cppcheck/addons/misra.py -generate-table > tests/misra/coverage_table

printf "\nPANDA F4 CODE\n"
cppcheck -DPANDA -DSTM32F4 -UPEDAL -DCAN3 -DUID_BASE \
         --suppressions-list="$SUPPRESSIONS_LIST" --suppress=*:*inc/* \
         -I $PANDA_DIR/board/ --dump --enable=all --inline-suppr --force \
         $PANDA_DIR/board/main.c 2>"$MISRA_OUTPUT_DIR/cppcheck_f4_output.txt"

python "$MISRA_ADDON" --no-summary "${MISRA_SUPPRESS_ARGS[@]}" $PANDA_DIR/board/main.c.dump 2> "$MISRA_OUTPUT_DIR/misra_f4_output.txt" || true

# strip (information) lines
cppcheck_f4_output=$( cat "$MISRA_OUTPUT_DIR/cppcheck_f4_output.txt" | grep -v ": information: " ) || true
misra_f4_output=$( cat "$MISRA_OUTPUT_DIR/misra_f4_output.txt" | grep -v ": information: " ) || true


printf "\nPANDA H7 CODE\n"
cppcheck -DPANDA -DSTM32H7 -UPEDAL -DUID_BASE \
         --suppressions-list="$SUPPRESSIONS_LIST" --suppress=*:*inc/* \
         -I $PANDA_DIR/board/ --dump --enable=all --inline-suppr --force \
         $PANDA_DIR/board/main.c 2>"$MISRA_OUTPUT_DIR/cppcheck_h7_output.txt"

python "$MISRA_ADDON" --no-summary "${MISRA_SUPPRESS_ARGS[@]}" $PANDA_DIR/board/main.c.dump 2> "$MISRA_OUTPUT_DIR/misra_h7_output.txt" || true

# strip (information) lines
cppcheck_h7_output=$( cat "$MISRA_OUTPUT_DIR/cppcheck_h7_output.txt" | grep -v ": information: " ) || true
misra_h7_output=$( cat "$MISRA_OUTPUT_DIR/misra_h7_output.txt" | grep -v ": information: " ) || true


printf "\nPEDAL CODE\n"
cppcheck -UPANDA -DSTM32F2 -DPEDAL -UCAN3 \
         --suppressions-list="$SUPPRESSIONS_LIST" --suppress=*:*inc/* \
         -I $PANDA_DIR/board/ --dump --enable=all --inline-suppr --force \
         $PANDA_DIR/board/pedal/main.c 2>"$MISRA_OUTPUT_DIR/cppcheck_pedal_output.txt"

python "$MISRA_ADDON" --no-summary "${MISRA_SUPPRESS_ARGS[@]}" $PANDA_DIR/board/pedal/main.c.dump 2> "$MISRA_OUTPUT_DIR/misra_pedal_output.txt" || true

# strip (information) lines
cppcheck_pedal_output=$( cat "$MISRA_OUTPUT_DIR/cppcheck_pedal_output.txt" | grep -v ": information: " ) || true
misra_pedal_output=$( cat "$MISRA_OUTPUT_DIR/misra_pedal_output.txt" | grep -v ": information: " ) || true

if [[ -n "$misra_f4_output" ]] || [[ -n "$cppcheck_f4_output" ]]
then
  echo "Failed! found Misra violations in panda F4 code:"
  echo "$misra_f4_output"
  echo "$cppcheck_f4_output"
  ERROR_CODE=1
fi

if [[ -n "$misra_h7_output" ]] || [[ -n "$cppcheck_h7_output" ]]
then
  echo "Failed! found Misra violations in panda H7 code:"
  echo "$misra_h7_output"
  echo "$cppcheck_h7_output"
  ERROR_CODE=1
fi

if [[ -n "$misra_pedal_output" ]] || [[ -n "$cppcheck_pedal_output" ]]
then
  echo "Failed! found Misra violations in pedal code:"
  echo "$misra_pedal_output"
  echo "$cppcheck_pedal_output"
  ERROR_CODE=1
fi

if [[ $ERROR_CODE > 0 ]]
then
  exit 1
fi

echo "Success"
