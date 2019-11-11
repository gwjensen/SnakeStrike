#!/bin/bash
########################################################################
## This script runs hyde if the build flag is passed, otherwise it does
## a verification of the current documents to the code base to make sure
## everything is up to date.
## @author gwjensen
##
## :TRICKY: This file has some hard-coded dependencies in terms of output directories
## and general directory structure. If you add new folders to the source, you'll need
## to update proper_output_dir() to reflect those directories.
##
## :NOTE: It seems like some typedefs don't get caught by hyde for some reason. You should
## double check if any were missed and add them manually.
##

usage()
{
prog_name=$(basename $0)
cat <<EOF
Usage: $prog_name -d directory_for_hyde [-c|-u] [-f]

      -d directory_for_hyde
          This is the pull path to the directory where the hyde executable can
          be found. It is usually something like '<path>/hyde/build'

      -c [optional]
          This asks hyde to check that the source and the documentation agree.
          This is the default action.

      -u [optional]
          This asks hyde to update the documentation with the new source info.
          Note: This will modify all files. It may not overwrite everything, but it
                will at the very least re-order lines in the files so they all must
                be re-committed. Use this with caution!!

      -f path_to_specific_file
         This will run hyde on only the specific file that is passed in. This is
         useful when adding new libraries and one doesn't want to re-run hyde on
         the already existing documentation. Otherwise the default is to run it
         on all files in the given directory.
EOF
    exit 1;
}

##################################################################
## This function gives the correct directory path for the api documentation when
## a full path filename is passed in.
## We put all non-object associated functions in a folder for ease of processing.
proper_output_dir()
{

    dest_dir=""
    if [[ "$1" == *"/common_source/image/"* ]]; then
        dest_dir="/common_source/image"
    elif [[ "$1" == *"/common_source/io/"* ]];then
        dest_dir="/common_source/io"
    elif [[ "$1" == *"/common_source/processing/"* ]];then
        dest_dir="/common_source/processing"
    elif [[ "$1" == *"/common_source/utils/"* ]];then
        dest_dir="/common_source/utils"
    elif [[ "$1" == *"/common_source/visualization/"* ]];then
        dest_dir="/common_source/visualization"
    elif [[ "$1" == *"/common_source/"* ]];then
        dest_dir="/common_source/free_functions"
    elif [[ "$1" == *"/opensource/EPVH/"* ]];then
        dest_dir="/opensource/EPVH"
    elif [[ "$1" == *"/opensource/Multiview3dRecon-KanataniBook/"* ]];then
        dest_dir="/opensource/Multiview3dRecon-KanataniBook"
    elif [[ "$1" == *"/opensource/munkres/"* ]];then
        dest_dir="/opensource/munkres"
    elif [[ "$1" == *"/opensource/"* ]];then
        dest_dir="/opensource/free_functions"
    elif [[ "$1" == *"/projects/gui/"* ]];then
        dest_dir="/gui"
    elif [[ "$1" == *"/projects/plugins/basler/"* ]];then
        dest_dir="/plugins/basler"
    elif [[ "$1" == *"/projects/plugins/correspondence/"* ]];then
        dest_dir="/plugins/correspondence"
    else
        printf "Error : File doesn't have expected directory structure.\n"
        exit 2;
    fi
    echo $dest_dir
}

##################################################################
## When given a path to a file it returns the filename without the
## last suffix.
##
get_filename_no_suffix()
{
    echo "$1" | sed -r "s/.+\/(.+)\..+/\1/"
}

#################################################################
## When creating a new subfolder for the API inside this script, we
## need to populate it with an index file. Pass in two strings, the
## first one for the parent folder and the second the full path of
## where to store the output file.
##
create_generic_index()
{
output_file="${2}/index.md"
cat >${output_file} <<EOF
---
layout: library
title: $1
owner: gwjensen
brief:
tags:
 - library
library-type: library
---
EOF
}

##################################################################
## Does a search of the repo from the build directory to find all subdirectories
## that could contain source files we would want hyde to process.
##
get_include_dirs()
{
    dir_array=()
    while IFS=  read -r -d $'\0'; do
        string='My long string'
        if [[ "$REPLY" != *"/build/"* ]]; then
            dir_array+=("$REPLY")
        fi
    done < <(find ../ -type d -not -path '*/build*' -not -path '*/docs*' -not -path '*.git*' -print0)
    include_list=""
    for dirn in "${dir_array[@]}"; do
        include_list="$include_list -I$PWD/$dirn"
    done
    include_list="-fpic $include_list -I/usr/local/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/include/vtk-5.10/ -I/opt/QT/5.7/gcc_64/include/ -I/opt/QT/5.7/gcc_64/include/QtCore -I/opt/QT/5.7/gcc_64/include/QtWidgets -I/opt/QT/5.7/gcc_64/include/QtConcurrent -I/opt/QT/5.7/gcc_64/include/QtGui -I/opt/QT/5.7/gcc_64/include/QtOpenGL -I/opt/QT/5.7/gcc_64/include/QtSql -I/opt/QT/5.7/gcc_64/include/QtXml -I/opt/pylon5/include/"
    echo "${include_list}"
}

HYDE_DIR=""

#HYDE_TYPE: 1 = update, 2 = check
HYDE_TYPE=2
SPECIFIC_FILE=""
while getopts ":cuhd:f:" o; do
    case "${o}" in
        c)
            HYDE_TYPE=2
            ;;
        u)
            HYDE_TYPE=1
            ;;
        d)
            HYDE_DIR=${OPTARG}
            ;;
        f)
            SPECIFIC_FILE=${OPTARG}
            ;;
        h)
            usage
            ;;
        *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))

if [ -z "${HYDE_DIR}" ]; then
    usage
fi

header_array=()
if [ -z "${SPECIFIC_FILE}" ]; then
    while IFS=  read -r -d $'\0'; do
        string='My long string'
        if [[ "$REPLY" != *"/build/"* ]]; then
            header_array+=("$REPLY")
        fi
    done < <(find ../ -type f -name "*.h" -print0)
  else
    header_array=${SPECIFIC_FILE}
fi

include_dirs=$(get_include_dirs )
#:NOTE: HYDE_TYPE: 1 = update, 2 = check
if ((${HYDE_TYPE} == 1)); then
    printf "\n\n ARE YOU SURE? This may overwrite files in the directory. Use with caution.\n\n"
    sleep 5
    printf "\n\n Really? Ok, proceeding....\n\n"
    sleep 5
    for fn in "${header_array[@]}"; do
        echo "update: Processing file $fn"
        dest_dir="$PWD/../docs/libraries$(proper_output_dir $fn)"
        echo "Destination dir: '$dest_dir'\n"
        if [ ! -d "$dest_dir" ]; then
          # :NOTE: Control will enter here if $DIRECTORY doesn't exist, but also if it
          # is a file. We just assume that won't be the case.
          printf "Creating new branch directory: '$dest_dir'\n"
          mkdir $dest_dir
          create_generic_index $(basename $dest_dir) $dest_dir
        fi

        file_dir=$(get_filename_no_suffix $fn)

        #:NOTE: If the structure of a subdirectory isn't updated to reflect that structure in
        ## proper_output_dir() then it will all be squished into one directory and thus one page.
        # The alternative is that we get a very nested class/function structure that requires
        # manually updating intermediate index.md files by hand. We don't want this as it breaks
        # the code consistency checks hyde can do.
        output_path="${dest_dir}"

        printf "Output path for this file is '$output_path'\n"

        cmnd="${HYDE_DIR}/hyde $fn -hyde-yaml-dir=$output_path -hyde-update -- -x c++ $include_dirs"
        $cmnd
        ret_code=$?
        if ((${ret_code} != 0)); then
            printf "Error : [%d] when executing hyde-update: '$cmnd'\n" $ret_code
            exit $ret_code;
        fi
        printf "\n\n"
    done
  else
      for fn in "${header_array[@]}"; do
          echo "check: Processing file $fn"
          dest_dir="$PWD/../docs/libraries$(proper_output_dir $fn)"
          file_dir=$(get_filename_no_suffix $fn)

          #:NOTE: If the structure of a subdirectory isn't updated to reflect that structure in
          ## proper_output_dir() then it will all be squished into one directory and thus one page.
          # The alternative is that we get a very nested class/function structure that requires
          # manually updating intermediate index.md files by hand. We don't want this as it breaks
          # the code consistency checks hyde can do.
          output_path="${dest_dir}"
          cmnd="${HYDE_DIR}/hyde $fn -hyde-yaml-dir=$output_path -hyde-validate -- -x c++ $include_dirs"
          $cmnd
          ret_code=$?
          if ((${ret_code} != 0)); then
              printf "Error : [%d] when executing hyde-update: '$cmnd'\n" $ret_code
              exit $ret_code;
          fi
          printf "\n\n"
      done
fi
