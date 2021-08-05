#!/bin/bash

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)"

function _symlink {
  echo 'Symlinking '"${1}"' to '"${2}"'...'
  if [[ "${3}" == "root" ]]; then
    sudo ln -s "${1}" "${2}"
  else
    ln -s "${1}" "${2}"
  fi
}

function symlink_file  {
  if [[ ! -e "${2}" ]]; then
    if [[ -L "${2}" ]]; then
      echo 'Removing broken link '"${2}"'...'
      sudo rm -f "${2}"
    fi
    _symlink "${@}"
  fi
}

# Symlink tmuxinator config
if [[ ! -d "${HOME}/.tmuxinator" ]]; then
  mkdir -p "${HOME}/.tmuxinator"
fi

for file in $(ls config/mux); do
  dest="${HOME}/.tmuxinator/${file}"
  orig="${DIR}/config/mux/${file}"
  symlink_file "${orig}" "${dest}"
done