#!/bin/bash
# Fetches the Gazebo object models used by gen_world.py into $VLMNAV_BASE/world_models (public model collection osrf/gazebo_models; sparse checkout of the needed directories).
# The list of models is read from code/gen_world.py (CATALOG). Needs git and internet access.
set -eu
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
HERE=$(cd "$(dirname "$0")" && pwd)
MODELS=$(cd "$HERE/../code" && python3 -c "import gen_world; print(' '.join(sorted(gen_world.CATALOG)))")
GAZEBO_MODELS_COMMIT=${GAZEBO_MODELS_COMMIT:-}   # the commit of the collection used in the paper was not recorded; world_models.sha256 identifies the files that were used
TMP=$(mktemp -d)
git clone --filter=blob:none --sparse https://github.com/osrf/gazebo_models.git "$TMP/gm"
git -C "$TMP/gm" sparse-checkout set $MODELS
[ -n "$GAZEBO_MODELS_COMMIT" ] && git -C "$TMP/gm" checkout "$GAZEBO_MODELS_COMMIT"
mkdir -p "$BASE/world_models"
for m in $MODELS; do cp -r "$TMP/gm/$m" "$BASE/world_models/"; done
rm -rf "$TMP"
echo "models copied to $BASE/world_models:"; ls "$BASE/world_models"
if [ -f "$HERE/world_models.sha256" ]; then
  if ( cd "$BASE/world_models" && sha256sum -c --quiet "$HERE/world_models.sha256" ); then
    echo "OK: the model files are identical to those used in the paper"
  else
    echo "WARNING: some model files differ from the version used in the paper (the collection has changed); object appearance or size may differ slightly"
  fi
else
  echo "NOTE: containers/world_models.sha256 not found; the version of the models cannot be verified"
fi
