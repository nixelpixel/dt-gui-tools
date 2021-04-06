#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# constants
JUPYTER_WS="/jupyter_ws"
JUPYTER_CMD="jupyter notebook --NotebookApp.token=''"

# configure environment
set +e
umask g+r,g+w

# create directory if it does not exist
if [ ! -d "${JUPYTER_WS}" ]; then
    mkdir -p "${JUPYTER_WS}"
fi
cd "${JUPYTER_WS}"

# check volume
mountpoint -q "${JUPYTER_WS}"
if [ $? -ne 0 ]; then
    echo "WARNING: The path '${JUPYTER_WS}' is not a VOLUME. All the changes will be deleted with the container."
    dt-exec ${JUPYTER_CMD}
else
    # from this point on, if something goes wrong, exit
    set -e

    # get _USERID of the ws dir
    _USERID=$(stat -c %u "${JUPYTER_WS}")
    _USERNAME='jupyter'

    # check if we have a user with that ID already
    if [ ! "$(getent passwd "${_USERID}")" ]; then
      echo "Creating a user '${_USERNAME}' with UID:${_USERID}"
      # create user
      useradd --create-home --uid ${_USERID} ${_USERNAME}
    else
      USER_STR=$(getent group ${_USERID})
      readarray -d : -t strarr <<< "$USER_STR"
      _USERNAME="${strarr[0]}"
      echo "A user with UID:${_USERID} (i.e., ${_USERNAME}) already exists. Reusing it."
    fi

    # launching app
    sudo -u "${_USERNAME}" bash -c "${JUPYTER_CMD}"
fi

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
