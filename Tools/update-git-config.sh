#!/bin/sh
#
# update-git-config.sh
#
# Automatically update the local name and email used by git.
#
# Description
# ===========
# It reads the remote push url and tries to extract the SCIPER number from the
# url. If the url does not match the pattern, the user is asked to enter its
# SCIPER number manually.
#
# Once the SCIPER number is known, a connection is made to the LDAP server in
# order to retrieve the mail and display name corresponding to the SCIPER
# number. If the server is unreachable or some required tools are not
# installed, the user is asked to enter the missing information manually.
#
# When the name and email of the user are known, the two following keys are
# updated in the local git configuration:
#  * user.email
#  * user.name
#
# Recommended tools
# =================
# The following tools are recommended in order to ensure that the script runs
# without any user input.
#
# cURL is required for the connection to the LDAP server. If cURL is not
# available, a warning message is displayed and the user is asked to enter its
# name and email manually.
#
# base64 from the coreutils package, openssl or python are required for
# decoding base64 encoded strings. If none of these tools are available, the
# user is asked to enter its name and/or email manually, depending on which
# data is encoded.
#

# Configuration
## The name of the remote to be read. If the remote does not exist, the user
## is asked to enter it manually.
REMOTE_NAME="origin"
## Hostname and protcol of the ldap server
LDAP_URL_PREFIX='ldap://ldap.epfl.ch/'
## A ldap url search template where %s is the SCIPER
LDAP_SEARCH_TEMPLATE='o=epfl,c=ch?mail,displayName?sub?(uniqueIdentifier=%s)'

# Function that attempt to decode a base64 encoded string
## If base64 is not available, it falls back to openssl or python
base64_decode()
{
    # First attempt using base64 utility
    PLAIN=`echo $1 | base64 -d 2>/dev/null`
    STATUS=$?
    if [ $STATUS -eq 0 ]; then
        echo "$PLAIN"
        return 0
    fi
    # Second attempt using openssl
    PLAIN=`echo $1 | openssl enc -base64 -d 2>/dev/null`
    STATUS=$?
    if [ $STATUS -eq 0 ]; then
        echo "$PLAIN"
        return 0
    fi
    # Last attempt using python
    PLAIN=`echo $1 | python -m base64 -d 2>/dev/null`
    STATUS=$?
    if [ $STATUS -eq 0 ]; then
        echo "$PLAIN"
        return 0
    fi
    # Could not decode base64 encoded argument
    return 1
}

# Check that we are inside a git repository
if [ ! -d .git ] && ! git rev-parse --git-dir >/dev/null 2>&1; then
    printf 'Not inside a git repository. Aborting.\n'
    exit 10
fi

# Exit if the user.name and user.email keys are already set locally
if git config --local --get user.name >/dev/null \
    && git config --local --get user.email >/dev/null; then
    exit 0
fi

# Check that a remote named REMOTE_NAME exists.
## `git remote` output a list of existing remotes
## `grep -q -x` quietly match a line which is exactly REMOTE_NAME
while ! git remote | grep -q -x "$REMOTE_NAME"; do
    printf 'No remote named "%s".\n' "$REMOTE_NAME"
    printf 'Available remotes are:\n'
    # Print a list of remotes
    git remote | sed 's:^:  * :'
    printf 'Please input a remote name manually or press enter to skip: '
    read REMOTE_NAME
    if [ -z "$REMOTE_NAME" ]; then
        printf 'No remote name provided. Aborting.\n'
        exit 20
    fi
done

# Retrieve the push url from the remote and extract the sciper number
## `git remote get-url --push origin` output the push url
## `sed ...` extract and output the sciper number or nothing if it doesnt match
SCIPER=`git remote get-url --push "$REMOTE_NAME" \
    | sed -n 's:^.*dronecourse-\([[:digit:]]\{1,\}\)\(\.git\)\{0,1\}$:\1:p'`
if [ -z "$SCIPER" ]; then
    printf 'Could not extract SCIPER number from push url of remote "%s".\n' \
        "$REMOTE_NAME"
    printf 'Please input your SCIPER number or press enter to skip: '
    read SCIPER
    if [ -z "$SCIPER" ]; then
        printf 'No SCIPER number provided.\n'
    fi
fi

if [ "$SCIPER" ]; then
    # Check that cURL is available
    if command -v curl >/dev/null; then
        # Build LDAP url and retrieve information from LDAP server
        LDAP_URL=`printf "$LDAP_URL_PREFIX$LDAP_SEARCH_TEMPLATE" "$SCIPER"`
        LDAP_REP=`curl -s "$LDAP_URL"`
        if [ ! -z "$LDAP_REP" ]; then
            # Extract mail and display name
            EMAIL_PLAIN=`echo "$LDAP_REP" | sed -n 's/^[[:space:]]*mail:\{1\} \([^[:space:]]*\)$/\1/p'`
            EMAIL_BASE64=`echo "$LDAP_REP" | sed -n 's/^[[:space:]]*mail:\{2\} \([^[:space:]]*\)$/\1/p'`
            if [ ! -z "$EMAIL_PLAIN" ]; then
                EMAIL="$EMAIL_PLAIN"
            elif [ ! -z "$EMAIL_BASE64" ]; then
                EMAIL=`base64_decode "$EMAIL_BASE64"`
                STATUS=$?
                if [ $STATUS -ne 0 ]; then
                    printf 'Could not decode base64 encoded string.\n'
                fi
            fi
            NAME_PLAIN=`echo "$LDAP_REP" | sed -n 's/^[[:space:]]*displayName:\{1\} \(.*\)$/\1/p'`
            NAME_BASE64=`echo "$LDAP_REP" | sed -n 's/^[[:space:]]*displayName:\{2\} \(.*\)$/\1/p'`
            if [ ! -z "$NAME_PLAIN" ]; then
                NAME="$NAME_PLAIN"
            elif [ ! -z "$NAME_BASE64" ]; then
                NAME=`base64_decode "$NAME_BASE64"`
                STATUS=$?
                if [ $STATUS -ne 0 ]; then
                    printf 'Could not decode base64 encoded string.\n'
                fi
            fi
        else
            printf 'Could not find any record for SCIPER %s.\n' "$SCIPER"
            printf ' * Debug info:\n'
            printf ' *   LDAP_URL_PREFIX:      %s\n' "$LDAP_URL_PREFIX"
            printf ' *   LDAP_SEARCH_TEMPLATE: %s\n' "$LDAP_SEARCH_TEMPLATE"
            printf ' *   LDAP_URL:             %s\n' "$LDAP_URL"
        fi
    else
        printf 'cURL is not installed. Data will not be fetched from LDAP server.\n'
    fi
fi

# Check if EMAIL and NAME are set
if [ -z "$EMAIL" ]; then
    printf 'Please input your mail address: '
    read EMAIL
fi
if [ -z "$NAME" ]; then
    printf 'Please input your name: '
    read NAME
fi

# Update local git config
git config --local user.name "$NAME"
git config --local user.email "$EMAIL"
