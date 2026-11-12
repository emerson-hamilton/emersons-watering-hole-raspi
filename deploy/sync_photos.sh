#!/usr/bin/bash
# Syncs motion-capture photos to the remote gallery server, then removes
# each file locally once rsync confirms it was successfully transferred.
#
# Install to: /home/<pi-user>/Naturebytes/Scripts/sync_photos.sh
# Fill in REMOTE_USER and REMOTE_HOST on the Pi; keep real values out of git.
set -uo pipefail

LOCAL_DIR="/mnt/usbdrive/img"
REMOTE_USER="<username>"                 # <-- change me
REMOTE_HOST="<hostname>"                 # <-- change me
SSH_KEY="/home/<pi-user>/.ssh/<upload-key>"

LOG_PREFIX="[emcam-sync]"

echo "${LOG_PREFIX} Starting sync..."

if rsync -a --remove-source-files \
    -e "ssh -i ${SSH_KEY} -o StrictHostKeyChecking=accept-new" \
    "${LOCAL_DIR}/" \
    "${REMOTE_USER}@${REMOTE_HOST}:"; then   # the server's authorized_keys entry runs rrsync, which maps this to .../emersoncam/uploads

    echo "${LOG_PREFIX} Sync succeeded."

    # --remove-source-files only deletes files, not the now-empty date
    # folders left behind - clean those up separately. -mmin +1 protects a
    # folder that capture_photo() has just created but not yet written to.
    find "${LOCAL_DIR}" -mindepth 1 -type d -empty -mmin +1 -delete

    echo "${LOG_PREFIX} Done."
else
    STATUS=$?
    echo "${LOG_PREFIX} Sync completed with errors (exit ${STATUS}) - any files that failed to transfer were left in place for the next run." >&2
    exit "${STATUS}"
fi
