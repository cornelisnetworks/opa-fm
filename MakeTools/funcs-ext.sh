#!/bin/bash -v
# BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2015-2017, Intel Corporation
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor the names of its contributors
#       may be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# END_ICS_COPYRIGHT8   ****************************************

# Override default funcs.sh functions from $ICSBIN.

#Following taken from funcs.sh
# These are included here to permit rpm builds by customers without needing
# our devtools (which includes funcs.sh)
# to aid tracking, if a change is needed for any of these functions
# move the given function toward the bottom of the script after the
# "END TAKEN FROM FUNCS.SH" line and then make the necessary edits
# this way it will be clear which functions should track devtools and which
# represent possibly product or generation specific adjustments
# This way this script will also have only 1 copy of each function and
# hence will avoid confusion to others reading it

#---------------------------------------------------------------------------
# Function:	toupper
# In Script:	funcs.sh
# Arguments:	a string to convert to upper case
# Description:	converts the given string to upper case
#			and returns the result to stdout
#			only alphabetic characters are affected
# Uses:
#	None
#---------------------------------------------------------------------------
function toupper()
{
    echo "$1"|tr '[a-z]' '[A-Z]'
} # end function

#---------------------------------------------------------------------------
# Function:	getBuildPlatform
# In Script:	funcs.sh
# Arguments:	none
# Description:	echo only the first six characters of the system name
# Uses:
#	None
#---------------------------------------------------------------------------                           
function getBuildPlatform()
{
     echo `uname -s | cut -c1-6`
} # end function

#---------------------------------------------------------------------------
# Function:	findTopDir
# In Script:	funcs.sh
# Arguments:	$1 --silent
# Description:	return top level directory
#               if --silent, the if top level not found then return $PWD
#               else return an error message
# Uses:
#	None
#---------------------------------------------------------------------------
function findTopDir()
{
    if   [ `getBuildPlatform` == "CYGWIN" ]; then
	tl_dir=`cygpath -w $PWD | sed 's/\\\/\//g'`
    else
	tl_dir=`pwd`
    fi
    while [ ! -f "$tl_dir/Makerules/Makerules.global" ]
    do
    	tl_dir=`dirname $tl_dir`
    	if [ "$tl_dir" == "/" -o "$tl_dir" == "." ]
    	then
            if [ "$1" == "--silent" ]; then               
                return
            else
                echo "Unable to locate TL_DIR, Makerules not found" >&2
    		return
            fi 
    	fi
    done
    
    echo $tl_dir
} # end function

#---------------------------------------------------------------------------
# Function:	settl
# In Script:	funcs.sh
# Arguments:
# Description:	find the top level directory, and set TL_DIR, else
#               set TL_DIR to the currect directory
# Uses:
#	None
#---------------------------------------------------------------------------
function settl()
{
      
	export TL_DIR=`findTopDir --silent`
} # end function

#---------------------------------------------------------------------------
# Function:	setwindbase
# In Script:	funcs.sh
# Arguments:	directory for WIND_BASE
# Description:	sets WIND_BASE and related variables
#		WIND_BASE, WIND_HOST_TYPE, WRS_HOST, WRS_BIN,
#		MIPS_BIN, MIPS_LIB, MIPS_PATH, UWIND_BASE, WIN_WRS_BIN
# Uses:         
#       None
#---------------------------------------------------------------------------
setwindbase()
{
    #### WindRiver Tornado 2 Environment Variables
            ## we compute UWIND_BASE and WIN_WRS_BIN here to save time
            ## during make (ie. it doesn't have to be recomputed in each makefile)
	    
   
    if [ "$1" == "" ]; then
        export WIND_BASE=
	return
    fi
    
	    
    if   [ `getBuildPlatform` == "CYGWIN" ]; then
	
	export WIND_BASE=`cygpath -w $1 | sed 's/\\\/\//g'`
	export WIND_HOST_TYPE=x86-win32

    else

	export WIND_BASE=$1		
	export WIND_HOST_TYPE=unix
    fi
    
    export WRS_HOST=$WIND_BASE/host/$WIND_HOST_TYPE
    export WRS_BIN=$WRS_HOST/bin

} # end function                      

#---------------------------------------------------------------------------
# Function:	isAllEmb
# In Script:	funcs.sh
# Arguments:
# Description:	determine if environment is an embedded checkout
# Uses:
#	None
#---------------------------------------------------------------------------
function isAllEmb()
{
	if [ "$TL_DIR" == "" ]; then
		settl
	fi

	[ `ls -d $TL_DIR/*_Firmware 2>/dev/null|wc -l` -ne 0 ]
} # end function

#---------------------------------------------------------------------------
# Function:	setbsp
# In Script:	funcs.sh
# Arguments:	none
# Description:	show available bsp's for TL_DIR project.
# Uses:
#	None
#---------------------------------------------------------------------------
function setbsp()
{
    local x=0
    local topLevelDir
    local the_selection

    if [ "$1" == "" ]; then
        if [ "$TL_DIR" == "" ]; then
            topLevelDir=`findTopDir`
        else
            topLevelDir=$TL_DIR
        fi
                
        SUPPORTED_BSP_FILE=""
        if [ -f "$topLevelDir/SUPPORTED_TARGET_BSPS" ]; then                
            SUPPORTED_BSP_FILE="$topLevelDir/SUPPORTED_TARGET_BSPS"
        fi
        if [ -f "$topLevelDir/$PROJ_FILE_DIR/SUPPORTED_TARGET_BSPS" ]; then                
            SUPPORTED_BSP_FILE="$topLevelDir/$PROJ_FILE_DIR/SUPPORTED_TARGET_BSPS"
        fi
        if [ "$SUPPORTED_BSP_FILE" != "" ]; then
            for i in `cat $SUPPORTED_BSP_FILE`
            do       
                echo "$(( x += 1 )) ) $i"
            done
            
            printf "choice: "
            read the_selection
            
            export TARGET_BSP=`cut -d" " -f$the_selection $SUPPORTED_BSP_FILE`
	    if [ "$TL_DIR" != "" ]; then
	    	echo $TARGET_BSP > $TL_DIR/.defaultBSP
	    fi
        else
            echo "Can not find SUPPORTED_TARGET_BSPS file in TL_DIR ($TL_DIR) or in a Firmware directory - try doing setct first"
        fi
    else
        export TARGET_BSP=$1
		if [ "$TL_DIR" != "" ]; then
			echo $TARGET_BSP > $TL_DIR/.defaultBSP
		fi
	fi
	if [ "$TARGET_BSP" != "" ]; then
		if [ "$PROJ_FILE_DIR" != "" ]; then
			grep -qsw "$TARGET_BSP" "$TL_DIR/$PROJ_FILE_DIR/SUPPORTED_TARGET_BSPS"
		else
			grep -qsw "$TARGET_BSP" "$TL_DIR/SUPPORTED_TARGET_BSPS"
		fi

		if [ $? -ne 0 ] ; then
			echo "Warning: '$TARGET_BSP' does not appear do be a supported BSP (using it anyway)"
		fi
    fi
    
} # end function

#---------------------------------------------------------------------------
# Function:	resetct
# In Script:	funcs.sh
# Arguments:	none
# Description:	clear card type for ALL_EMB builds
# Uses:
#	None
#---------------------------------------------------------------------------
function resetct()
{
    unset CARD_TYPE
    unset PROJ_FILE_DIR
	if [ "$TL_DIR" != "" ]; then
		rm -f $TL_DIR/.defaultCT
	fi
    echo "CARD_TYPE has been reset"
}

#---------------------------------------------------------------------------
# Function:	targetos
# In Script:	funcs.sh
# Arguments:	1 = target operating system
# Description:	specify target operating system for make to build for
# Uses:
#	sets BUILD_TARGET_OS
#---------------------------------------------------------------------------
function targetos()
{
    local targetOsType=`toupper $1`

    if   [ `getBuildPlatform` == "CYGWIN" ]; then

	case $targetOsType in
	VXWORKS)
	    export BUILD_TARGET_OS=VXWORKS
	    ;;
	WIN32)
	    export BUILD_TARGET_OS=WIN32
	    ;;
	CYGWIN)
	    export BUILD_TARGET_OS=CYGWIN
	    ;;	
	*)
	    printf "Usage: targetos [cygwin|vxworks|win32]\n\n"
	    return
	    ;;
	esac
    elif   [ `getBuildPlatform` == "Darwin" ]; then
    	case $targetOsType in
	DARWIN)
	    export BUILD_TARGET_OS=DARWIN
	    ;;
	*)
	    printf "Usage: targetos [darwin]\n\n"
	    return
	    ;;
	esac
    else
    	case $targetOsType in
	VXWORKS)
	    export BUILD_TARGET_OS=VXWORKS
	    ;;
	LINUX)
	    export BUILD_TARGET_OS=LINUX
	    ;;
	*)
	    printf "Usage: targetos [linux]\n\n"
	    return
	    ;;
	esac
    fi
    
    echo "Build target operating system set to $BUILD_TARGET_OS"
} # end function                      



################################################################################
####END TAKEN FROM FUNCS.SH
################################################################################



# NOTE: This function changes often when environment variables are added
#       to display to end users.
#---------------------------------------------------------------------------
# Function:	showenv
# In Script:	funcs-ext.sh
# Arguments:	none
# Description:	show the intel cde environment settings
# Uses:         
#       None
#---------------------------------------------------------------------------
function showenv()
{
    local SMP

    printf "Intel CDE Environment Settings:\n\n"

    if [[ "$BUILD_TARGET" == "" || "$1" == "-h" ]]; then
        local TARGET_COMMENT="[use: target] "
    fi      
    if [[ "$BUILD_UNIT_TEST" == "" || "$1" == "-h" ]]; then
        local UNIT_TEST_COMMENT="[use: settest|setnotest] "
    fi
    if [[ "$BUILD_CONFIG" == "" || "$1" == "-h" ]]; then
        local CONFIG_COMMENT="[use: setdbg|setrel command] "
    fi
    if [[ "$BUILD_TARGET_OS_VERSION" == "" || "$1" == "-h" ]]; then
        local OS_COMMENT="[use: setver] "
    fi    	
    if [[ "$RELEASE_TAG" == "" || "$1" == "-h" ]]; then
        local RELEASE_TAG_COMMENT="[use: export RELEASE_TAG=R#] "
    fi    	
    
    echo "Build Target       : $TARGET_COMMENT$BUILD_TARGET"
    echo "Build Target OS    : $TARGET_COMMENT$OS_COMMENT$BUILD_TARGET_OS_VENDOR $BUILD_TARGET_OS $BUILD_TARGET_OS_VERSION $SMP"
    echo "Build Platform     : $TARGET_COMMENT$BUILD_PLATFORM_OS_VENDOR $BUILD_PLATFORM"
    echo "Build Target Tools : $TARGET_COMMENT$BUILD_TARGET_TOOLCHAIN"
    echo "Build Unit Test    : $UNIT_TEST_COMMENT$BUILD_UNIT_TEST"
    echo "Build Configuration: $CONFIG_COMMENT$BUILD_CONFIG"
    echo "Release Tag        : $RELEASE_TAG_COMMENT$RELEASE_TAG"
    
    if [ "$BUILD_TARGET_OS" == "VXWORKS" ]; then
        if [[ "$BUILD_TMS" == "" || "$1" == "-h" ]]; then
            local TMS_COMMENT="[use: export BUILD_TMS=(yes|no)] "
        fi
        if [[ "$TGT_SUBDIR_NAME" == "" || "$1" == "-h" ]]; then
            local SUBDIR_COMMENT="[use: export TGT_SUBDIR_NAME=(target|tmsTarget)] "
        fi
        if [[ "$EVAL_HARDWARE" == "" || "$1" == "-h" ]]; then
            local EVAL_COMMENT="[use: export EVAL_HARDWARE=(yes|no)] "
        fi
        if [[ "$NO_SSP" == "" || "$1" == "-h" ]]; then
            local SSP_COMMENT="[use: export NO_SSP=(1|0)] "
        fi		
        if [[ "$STOP_ON_ERROR" == "" || "$1" == "-h" ]]; then
            local STOP_ON_ERROR_COMMENT="[use: export STOP_ON_ERROR=(yes|no)] "
        fi
        if [[ "$TARGET_BSP" == "" || "$1" == "-h" ]]; then
            local BSP_COMMENT="[use: setbsp (`cat $TL_DIR/SUPPORTED_TARGET_BSPS 2> /dev/null`)] "
        fi         	    	
        if [[ "$WIND_BASE" == "" || "$1" == "-h" ]]; then
            local WIND_BASE_COMMENT="[use: target | setwindbase] "
        fi                
        local CARD_TYPE_COMMENT
        if isAllEmb; then
            if [[ "$CARD_TYPE" == "" || "$1" == "-h" ]]; then
                    CARD_TYPE_COMMENT="[use: setct] "
            fi	    
        else
            if [[ "$CARD_TYPE" != "" || "$1" == "-h" ]]; then
                    CARD_TYPE_COMMENT="[ALL_EMB only, use: resetct] "
            else
                    CARD_TYPE_COMMENT="[ALL_EMB only] "
            fi
        fi
        if [[ "$WEB_STANDALONE" == "" || "$1" == "-h" ]]; then
            local WEB_STANDALONE_COMMENT="[use: export WEB_STANDALONE=(1|0)] "
        fi 
                                       
        echo   "Build TMS          : $TMS_COMMENT$BUILD_TMS"
        echo   "Build Target Subdir: $SUBDIR_COMMENT$TGT_SUBDIR_NAME"
        echo   "EVAL HARDWARE      : $EVAL_COMMENT$EVAL_HARDWARE"		
        echo   "NO SSP             : $SSP_COMMENT$NO_SSP"	    
        echo   "STOP ON ERROR      : $STOP_ON_ERROR_COMMENT$STOP_ON_ERROR"       
        echo   "Target BSP         : $BSP_COMMENT$TARGET_BSP"      
        echo   "Tornado Base       : $WIND_BASE_COMMENT$WIND_BASE"
        echo   "Card Type          : $CARD_TYPE_COMMENT$CARD_TYPE"
        echo   "Web Standalone     : $WEB_STANDALONE_COMMENT$WEB_STANDALONE"
	else
        if [[ "$OFED_STACK_PREFIX" == "" || "$1" == "-h" ]]; then
            local OFED_STACK_PREFIX_COMMENT="[use: export OFED_STACK_PREFIX=/dir] "
        fi
        #if [[ "$BUILD_ULPS" == "" || "$1" == "-h" ]]; then
        #    local BUILD_ULPS_COMMENT="[use: setulps 'ulp list'] "
        #fi
        #if [[ "$BUILD_SKIP_ULPS" == "" || "$1" == "-h" ]]; then
        #    local BUILD_SKIP_ULPS_COMMENT="[use: setskipulps 'ulp list'] "
        #fi
        echo   "OFED Stack Prefix  : $OFED_STACK_PREFIX_COMMENT$OFED_STACK_PREFIX"
        #echo   "Build ULPs         : $BUILD_ULPS_COMMENT$BUILD_ULPS"
        #echo   "Build Skip ULPs    : $BUILD_SKIP_ULPS_COMMENT$BUILD_SKIP_ULPS"
    fi

    if [[ "$TL_DIR" == "" || "$1" == "-h" ]]; then
       local TL_DIR_COMMENT="[use: target] "
    fi 
    echo   "Top Level Directory: $TL_DIR_COMMENT$TL_DIR"

    if [[ "$PROJ_FILE_DIR" == "" || "$1" == "-h" ]]; then
        local PROJ_FILE_DIR_COMMENT="[use: export PROJ_FILE_DIR as needed] "
    fi      
    echo   "Proj File Dir      : $PROJ_FILE_DIR_COMMENT$PROJ_FILE_DIR"

    if [[ "$PRODUCT" == "" || "$1" == "-h" ]]; then
        local PRODUCT_COMMENT="[use: export PRODUCT as needed] "
    fi      
    echo   "Product            : $PRODUCT_COMMENT$PRODUCT"

} # end function        

setbsp ()
{
    local x=0;
    local topLevelDir;
    local the_selection;
    if [ "$1" == "" ]; then
        if [ "$TL_DIR" == "" ]; then
            topLevelDir=`findTopDir`;
        else
            topLevelDir=$TL_DIR;
        fi;
        SUPPORTED_BSP_FILE="";
        if [ -f "$topLevelDir/SUPPORTED_TARGET_BSPS" ]; then
            SUPPORTED_BSP_FILE="$topLevelDir/SUPPORTED_TARGET_BSPS";
        fi;
        if [ -f "$topLevelDir/$PROJ_FILE_DIR/SUPPORTED_TARGET_BSPS" ]; then
            SUPPORTED_BSP_FILE="$topLevelDir/$PROJ_FILE_DIR/SUPPORTED_TARGET_BSPS";
        fi;
        if [ "$SUPPORTED_BSP_FILE" != "" ]; then
            for i in `cat $SUPPORTED_BSP_FILE`;
            do
                echo "$(( x += 1 )) ) $i";
            done;
            printf "choice: ";
            read the_selection;
            export TARGET_BSP=`cut -d" " -f$the_selection $SUPPORTED_BSP_FILE`;
			export TARGET_BSP_UPCASE=`echo $TARGET_BSP | tr '[:lower:]' '[:upper:]'`
            if [ "$TL_DIR" != "" ]; then
                echo $TARGET_BSP >$TL_DIR/.defaultBSP;
            fi;
        else
            echo "Can not find SUPPORTED_TARGET_BSPS file in TL_DIR ($TL_DIR) or in a Firmware directory - try doing setct first";
        fi;
    else
        export TARGET_BSP=$1;
        export TARGET_BSP_UPCASE=`echo $TARGET_BSP | tr '[:lower:]' '[:upper:]'`
        if [ "$TL_DIR" != "" ]; then
            echo $TARGET_BSP >$TL_DIR/.defaultBSP;
        fi;
    fi;
    if [ "$TARGET_BSP" != "" ]; then
		export TARGET_BSP_UPCASE=`echo $TARGET_BSP | tr '[:lower:]' '[:upper:]'`
        if [ "$PROJ_FILE_DIR" != "" ]; then
            grep -qsw "$TARGET_BSP" "$TL_DIR/$PROJ_FILE_DIR/SUPPORTED_TARGET_BSPS";
        else
            grep -qsw "$TARGET_BSP" "$TL_DIR/SUPPORTED_TARGET_BSPS";
        fi;
        if [ $? -ne 0 ]; then
            echo "Warning: '$TARGET_BSP' does not appear do be a supported BSP (using it anyway)";
        fi;
    fi
}

function settarget()
{
    local targetType=`toupper $1`
    local message

    if [[ "$1" == "-h" ]]; then
        printf "Usage: target ia32|ia64|x86_64|em64t|mips|atom|ppc|ppc64\n"
        return
    fi
    
    export BUILD_PLATFORM_OS_VENDOR=`os_vendor`
    export BUILD_PLATFORM_OS_VENDOR_VERSION=`os_vendor_version $BUILD_PLATFORM_OS_VENDOR`    
    export BUILD_TARGET_OS_VENDOR=$BUILD_PLATFORM_OS_VENDOR
    export BUILD_TARGET_OS_VENDOR_VERSION=$BUILD_PLATFORM_OS_VENDOR_VERSION
   
    if   [ `getBuildPlatform` == "CYGWIN" ]; then

            if [ "$TL_DIR" == "" ]; then
                settl            
            fi
                    
            if [ "$WIND_BASE" == "" ]; then
                setwindbase $TL_DIR
            fi
       
	    isBSP=`toupper $2`                             
                                 
            case $targetType in
            MIPS)
                targetos VXWORKS       
                export MIPS_LIB=$WRS_HOST/lib/gcc-lib/mips-wrs-vxworks/cygnus-2.7.2-960126	    
                export MIPS_PATH=`cygpath -u $WRS_BIN 2> /dev/null`
                export BUILD_TARGET_TOOLCHAIN=GNU
                export BUILD_TARGET_OS_VERSION=5.4
                export BUILD_TARGET_OS_VENDOR=WindRiver
                export BUILD_TARGET=MIPS
                ;;
            X86|I386)
                targetos VXWORKS
                export I386_LIB=$WRS_HOST/lib/gcc-lib/i386-wrs-vxworks/cygnus-2.7.2-960126
                export I386_PATH=$WRS_BIN	
                export BUILD_TARGET_TOOLCHAIN=GNU
                export BUILD_TARGET_OS_VERSION=5.4
                export BUILD_TARGET=I386
                ;;
            CYGWIN)
                targetos CYGWIN
                export BUILD_TARGET_TOOLCHAIN=GNU
                export BUILD_TARGET=CYGWIN
                ;;
            WIN32)
                targetos WIN32
                export BUILD_TARGET_TOOLCHAIN=WIN32
                export BUILD_TARGET=WIN32
                ;;
            *)
                printf "Unknown target $1\n"
                printf "Usage: target [cygwin | mips | win32] [bsp]\n\n"
                return
                ;;	
            esac           
            
            if [ "$isBSP" == "BSP" ]; then
                message="for BSP development"
                    UWRS_BIN=`cygpath -u $WRS_BIN`
                    export WIND_BASE=`cygpath -w $WIND_BASE 2> /dev/null`
                export PATH=`update_path_start $PATH $UWRS_BIN $UWRS_BIN`
            fi    
    elif   [ `getBuildPlatform` == "Darwin" ]; then
        settl
        case $targetType in
        PPC)
            targetos DARWIN
            export BUILD_TARGET_TOOLCHAIN=GNU
            export BUILD_TARGET=PPC
            ;;
        *)		    
            if [ "$1" != "" ]; then
               printf "Unknown target $1\n"
            fi
            printf "Usage: target ppc\n\n"
            return
            ;;    
        esac
    else
    
        settl
        setwindbase $TL_DIR

        case $targetType in
		ATOM)
            targetos VXWORKS       
			export TGT_SUBDIR_NAME=target
			export TGT_DIR=$TL_DIR/target
            export ATOM_PATH=
            export ATOM_LIB=$TGT_DIR/lib/pentium/ATOM/common/libgcc.a
            export BUILD_TARGET_TOOLCHAIN=GNU
            export BUILD_TARGET_OS_VENDOR=WindRiver
            export BUILD_TARGET_OS_VERSION=6.9.4.6
            export BUILD_TARGET=ATOM
            export WIND_HOST_TYPE=x86-linux2
            if [ "$LD_LIBRARY_PATH" == "" ] ; then
            	export LD_LIBRARY_PATH=$TL_DIR/host/wrs/gnu/lib64
            else
                export LD_LIBRARY_PATH=$TL_DIR/host/wrs/gnu/lib64:$LD_LIBRARY_PATH
            fi
            export PATH=$PATH:$ICSLOCAL/Emb_Utils:$TL_DIR/host/wrs/gnu/4.3.3-vxworks-6.9/x86-linux2/bin:$TL_DIR/host/wrs/gnu/4.3.3-vxworks-6.9/x86-linux2/libexec/gcc/i586-wrs-vxworks/4.3.3

            ;;
        MIPS)
            targetos VXWORKS       
			export TGT_SUBDIR_NAME=tmsTarget
			export TGT_DIR=$TL_DIR/tmsTarget
            export MIPS_PATH=
            export MIPS_LIB=$TL_DIR/host/$WIND_HOST_TYPE/lib/gcc-lib/mips-wrs-vxworks/cygnus-2.7.2-960126/mips3/libgcc.a
            export BUILD_TARGET_TOOLCHAIN=GNU
            export BUILD_TARGET_OS_VENDOR=WindRiver
            export BUILD_TARGET_OS_VERSION=5.4
            export BUILD_TARGET=MIPS
            ;;
        LINUX)
            targetos LINUX
            export BUILD_TARGET_TOOLCHAIN=GNU
            export BUILD_TARGET=IA32
            ;;
        IA32)
            targetos LINUX
            export BUILD_TARGET_TOOLCHAIN=GNU
            export BUILD_TARGET=IA32
            ;;
        LINUX64)
            targetos LINUX
            export BUILD_TARGET_TOOLCHAIN=GNU
            export BUILD_TARGET=IA64
            ;;
        IA64)
            targetos LINUX
            export BUILD_TARGET_TOOLCHAIN=GNU
            export BUILD_TARGET=IA64
            ;;
	X86_64)
	    targetos LINUX
	    export BUILD_TARGET_TOOLCHAIN=GNU
	    export BUILD_TARGET=X86_64
	    ;;
	PPC64)
	    targetos LINUX
	    export BUILD_TARGET_TOOLCHAIN=GNU
	    export BUILD_TARGET=PPC64
	    ;;
	EM64T)
	    targetos LINUX
	    export BUILD_TARGET_TOOLCHAIN=GNU
	    export BUILD_TARGET=EM64T
	    ;;
        *)		    
            if [ "$1" != "" ]; then
               printf "Unknown target $1\n"
            fi
            printf "Usage: target ia32|ia64|x86_64|em64t|mips|atom|ppc64\n\n"
            return
            ;;    
        esac
    fi
	set_os_identifier

    echo "Build target set to $BUILD_TARGET $message"

    if isAllEmb && [[ "$BUILD_TARGET_OS" = "VXWORKS" ]]; then
       if [ -e "$TL_DIR/.defaultCT" ]; then
          export CARD_TYPE=`cat $TL_DIR/.defaultCT 2>&1 /dev/null`
          export PROJ_FILE_DIR=$CARD_TYPE"_Firmware"
          echo "Card type set to $CARD_TYPE"
       fi
    else
       resetct > /dev/null
    fi
    
    if [[ "$BUILD_TARGET_OS" = "VXWORKS" && -e "$TL_DIR/.defaultBSP" ]]; then
        export TARGET_BSP=`cat $TL_DIR/.defaultBSP 2>&1 /dev/null`
        export TARGET_BSP_UPCASE=`echo $TARGET_BSP | tr '[:lower:]' '[:upper:]'`
        echo "Target BSP is ${TARGET_BSP:-empty.}"
    fi        
    
    if [[ -e "$TL_DIR/.defaultBuildConfig" ]]; then
        export BUILD_CONFIG=`cat $TL_DIR/.defaultBuildConfig 2>&1 /dev/null`
        echo "Build Configuration is ${BUILD_CONFIG:-empty.}"
    fi        
    if [[ -e "$TL_DIR/.defaultBuildUlps" ]]; then
        export BUILD_ULPS=`cat $TL_DIR/.defaultBuildUlps 2>&1 /dev/null`
        echo "Build ULPs is ${BUILD_ULPS:-empty.}"
    else
        export BUILD_ULPS=${BUILD_ULPS:-all}
    fi        
    if [[ -e "$TL_DIR/.defaultBuildSkipUlps" ]]; then
        export BUILD_SKIP_ULPS=`cat $TL_DIR/.defaultBuildSkipUlps 2>&1 /dev/null`
        echo "Build Skip ULPs is ${BUILD_SKIP_ULPS:-empty.}"
    else
        export BUILD_SKIP_ULPS=${BUILD_SKIP_ULPS:-none}
    fi        
} # end function

# NOTE: This function is changed whenever we add different platform or
#       target support.
#---------------------------------------------------------------------------
# Function:	os_vendor
# In Script:	funcs-ext.sh
# Arguments:	none
# Description:	determine the os vendor based on build system
#---------------------------------------------------------------------------
function os_vendor()
{
    if [ `uname -s` == "Darwin" ]
    then
        # Apple Mac
        rval=apple
    else
        filelist=($('ls' /etc/*-release | egrep -v lsb | egrep -v os))
        rval=""
        if [ ${#filelist[@]} -eq 0 ] && [ -f /etc/lsb-release ]; then
            rval=$(cat /etc/lsb-release | egrep DISTRIB_ID | cut -d'=' -f2 | tr '[:upper:]' '[:lower:]')
        fi
        for file in $filelist
        do
	    if [ -f $file ]
	    then
		rval=$(basename $file -release)
		if [ $rval = 'SuSE' ]
		then
			if [ -f /etc/UnitedLinux-release ]
			then
				rval=UnitedLinux
			fi
		elif [ $rval = 'centos' ]
		then
			rval=redhat
		elif [ $rval != 'os' ]
		then
			break
		fi
	    fi
        done
    fi
    echo $rval
}

# NOTE: This function is changed whenever we add different platform or
#       target support.
#---------------------------------------------------------------------------
# Function:	os_vendor_version
# In Script:	funcs-ext.sh
# Arguments:	1 = os_vendor
# Description:	determine the os vendor release level based on build system
#---------------------------------------------------------------------------
function os_vendor_version()
{
	if [[ -e /etc/os-release ]]; then
		. /etc/os-release
		# - use VERSION_ID - it has a common format among distros 
		# - mimic old way and drop $minor if eq 0 (see redhat handling below)
		# - drop '.'(dot)
		if [ $1 = "ubuntu" ]; then
			rval=ES$(echo $VERSION_ID | sed -e 's/\.//')
		else
			rval=ES$(echo $VERSION_ID | sed -e 's/\.[0]//' -e 's/\.//')
        	fi
		echo $rval
		return
	fi

	# using old way if '/etc/os-release' not found
	case $1 in
	apple)
    	rval=`sw_vers -productVersion|cut -f1-2 -d.`
		;;
	rocks)
		rval=`cat /etc/rocks-release | cut -d' ' -f3`
		;;
	scyld)
		rval=`cat /etc/scyld-release | cut -d' ' -f4`
		;;
	mandrake)
		rval=`cat /etc/mandrake-release | cut -d' ' -f4`
		;;
	fedora)
		if grep -qi core /etc/fedora-release
		then
			rval=`cat /etc/fedora-release | cut -d' ' -f4`
		else
			rval=`cat /etc/fedora-release | cut -d' ' -f3`
		fi
		;;
	redhat)
		if grep -qi advanced /etc/redhat-release
		then
			rval=`cat /etc/redhat-release | cut -d' ' -f7`
		elif grep -qi enterprise /etc/redhat-release
		then
			# /etc/redhat-release = "Red Hat Enterprise Linux Server release $a.$b ($c)"
			rval="ES"`cat /etc/redhat-release | cut -d' ' -f7 | cut -d. -f1`
			major=`cat /etc/redhat-release | cut -d' ' -f7 | cut -d. -f1`
			minor=`cat /etc/redhat-release | cut -d' ' -f7 | cut -d. -f2`
			if [ \( $major -ge 7 -a $minor -ne 0 \) -o \( $major -eq 6 -a $minor -ge 7 \) ]
			then
				rval=$rval$minor
			fi
		elif grep -qi centos /etc/redhat-release
		then
			# CentOS 
			rval="ES"`cat /etc/redhat-release | sed -r 's/^.+([[:digit:]])\.([[:digit:]]).+$/\1\2/'`
		elif grep -qi scientific /etc/redhat-release
		then
			# Scientific Linux.
			rval="ES"`cat /etc/redhat-release | sed -r 's/^.+([[:digit:]])\.([[:digit:]]).+$/\1/'`
		else
			rval=`cat /etc/redhat-release | cut -d' ' -f5`
		fi
		;;
	UnitedLinux)
		rval=`grep United /etc/UnitedLinux-release | cut -d' ' -f2`
		;;
	SuSE)
		if grep -qi enterprise /etc/SuSE-release
		then
			rval="ES"`grep -i enterprise /etc/SuSE-release | cut -d' ' -f5`
		else
			rval=`grep -i SuSE /etc/SuSE-release | cut -d' ' -f3`
		fi
		;;
	turbolinux)
		rval=`cat /etc/turbolinux-release | cut -d' ' -f3`
		;;
	esac
	echo $rval
}

# NOTE: This function is changed whenever we add different platform or
#       target support.
#---------------------------------------------------------------------------
# Function:	os_identifier
# In Script:	funcs-ext.sh
# Arguments:	None
# Description:	generate concise ID for targer/vendor/major version
# Uses:
#	BUILD_TARGET, BUILD_TARGET_OS_VENDOR, BUILD_TARGET_OS_VENDOR_VERSION
#	sets BUILD_TARGET_OS_ID
function set_os_identifier()
{
	local arch ver
    arch=`echo $BUILD_TARGET | tr '[:upper:]' '[:lower:]'`
	case "$BUILD_TARGET_OS_VENDOR" in
	redhat)
		ver=RH$(echo $BUILD_TARGET_OS_VENDOR_VERSION|sed -e 's/ES/EL/' -e 's/\..*//');;
	SuSE)
		ver=SL$(echo $BUILD_TARGET_OS_VENDOR_VERSION|sed -e 's/\..*//');;
	*)
		ver="$BUILD_TARGET_OS_VENDOR.$BUILD_TARGET_OS_VENDOR_VERSION"''
	esac
	export BUILD_TARGET_OS_ID=$ver-$arch
}

# NOTE: This function is changed whenever we add different platform or
#       target support.
#---------------------------------------------------------------------------
# Function:	target
# In Script:	funcs-ext.sh
# Arguments:	1 = target platform
#               2 = bsp [optional] on cygwin build platform only
# Description:	specify target platform for make to build for
# Uses:
#	sets BUILD_TARGET
#       if bsp development is specified then export PATH to allow the
#       compiler to be used on the command line
#---------------------------------------------------------------------------
function target()
{
    local targetType=`toupper $1`
    local message

    # always set top level directoy
    settl
    
    export BUILD_PLATFORM_OS_VENDOR=`os_vendor`    
    export BUILD_PLATFORM_OS_VENDOR_VERSION=`os_vendor_version $BUILD_PLATFORM_OS_VENDOR`    
    export BUILD_TARGET_OS_VENDOR=$BUILD_PLATFORM_OS_VENDOR
    export BUILD_TARGET_OS_VENDOR_VERSION=$BUILD_PLATFORM_OS_VENDOR_VERSION

    #  Allows us to override functions that change often.
    if [[ "$TL_DIR" != "" && -e "$TL_DIR/MakeTools/funcs-ext.sh" ]]; then
        if [[ -e "$ICSBIN/funcs.sh" ]]; then
          echo "Resourcing default $ICSBIN/funcs.sh"
          . $ICSBIN/funcs.sh $*
          . $TL_DIR/MakeTools/funcs-ext.sh $*
          echo "Using $TL_DIR/MakeTools/funcs-ext.sh"
          settarget $*
        else
          echo "The development environment is not properly configured.  Please source ics.sh file."
          return
        fi
    else
    	if [[ "$1" == "-h" ]]; then
	    printf "Usage: target ia32|ia64|x86_64|em64t|mips|atom|ppc64\n"
	    return
	fi    
            
        echo "Using default functions."
        if   [ `getBuildPlatform` == "CYGWIN" ]; then
    
        	if [ "$TL_DIR" == "" ]; then
        	    settl            
        	fi
                	
        	if [ "$WIND_BASE" == "" ]; then
        	    setwindbase $TL_DIR
        	fi
           
                isBSP=`toupper $2`                             
                                     
        	case $targetType in
			ATOM)
                targetos VXWORKS       
				export TGT_SUBDIR_NAME=target
				export TGT_DIR=$TL_DIR/target
                export ATOM_PATH=
                export ATOM_LIB=$TGT_DIR/lib/pentium/ATOM/common/libgcc.a
                export BUILD_TARGET_TOOLCHAIN=GNU
                export BUILD_TARGET_OS_VENDOR=WindRiver
                export BUILD_TARGET_OS_VERSION=6.9.2
                export BUILD_TARGET=ATOM
            	export WIND_HOST_TYPE=x86-linux2
                ;;
        	MIPS)
        	    #if checkwindbase; then 
        	    #	return
        	    #fi
            
        	    targetos VXWORKS       
        	    export TGT_SUBDIR_NAME=tmsTarget
        	    export TGT_DIR=$TL_DIR/tmsTarget
         	    export MIPS_LIB=$WRS_HOST/lib/gcc-lib/mips-wrs-vxworks/cygnus-2.7.2-960126	    
        	    export MIPS_PATH=`cygpath -u $WRS_BIN 2> /dev/null`
                    export BUILD_TARGET_TOOLCHAIN=GNU
        	    export BUILD_TARGET_OS_VERSION=5.4
        	    export BUILD_TARGET=MIPS
        	    ;;
        	X86|I386)
        	    #if checkwindbase; then 
        	    #	return
        	    #fi
        	    
        	    targetos VXWORKS
        	    export I386_LIB=$WRS_HOST/lib/gcc-lib/i386-wrs-vxworks/cygnus-2.7.2-960126
        	    export I386_PATH=$WRS_BIN	
                    export BUILD_TARGET_TOOLCHAIN=GNU
        	    export BUILD_TARGET_OS_VERSION=5.4
        	    export BUILD_TARGET=I386
        	    ;;
        	CYGWIN)
        	    targetos CYGWIN
                    export BUILD_TARGET_TOOLCHAIN=GNU
        	    export BUILD_TARGET=CYGWIN
        	    ;;
        	WIN32)
        	    targetos WIN32
                    export BUILD_TARGET_TOOLCHAIN=WIN32
        	    export BUILD_TARGET=WIN32
        	    ;;
        	*)
        	    printf "Unknown target $1\n"
        	    printf "Usage: target [cygwin | mips | atom | win32] [bsp]\n\n"
        	    return
        	    ;;	
                esac           
                
                if [ "$isBSP" == "BSP" ]; then
                    message="for BSP development"
        	        UWRS_BIN=`cygpath -u $WRS_BIN`
        	        export WIND_BASE=`cygpath -w $WIND_BASE 2> /dev/null`
                    export PATH=`update_path_start $PATH $UWRS_BIN $UWRS_BIN`
                fi    
        elif   [ `getBuildPlatform` == "DARWIN" ]; then
            case $targetType in
            PPC)
                targetos DARWIN
                export BUILD_TARGET_TOOLCHAIN=GNU
                export BUILD_TARGET=PPC
                ;;
            *)		    
                if [ "$1" != "" ]; then
                   printf "Unknown target $1\n"
                fi
                printf "Usage: target ppc\n\n"
                return
            esac
        else
                    
            if [[ "$TL_DIR" == "" ]]; then
               echo "Unable to determine top level directory."
               echo "The top level directory must have a ./Makerules/Makerules.global file."
               return  
            fi
            
            setwindbase $TL_DIR
    
            case $targetType in
            MIPS)
                targetos VXWORKS       
                export MIPS_PATH=
                export MIPS_LIB=$TL_DIR/host/$WIND_HOST_TYPE/lib/gcc-lib/mips-wrs-vxworks/cygnus-2.7.2-960126/mips3/libgcc.a
                export BUILD_TARGET_TOOLCHAIN=GNU
                export BUILD_TARGET_OS_VERSION=5.4
                export BUILD_TARGET=MIPS
                ;;
            LINUX)
                targetos LINUX
                export BUILD_TARGET_TOOLCHAIN=GNU
                export BUILD_TARGET=IA32
                ;;
            IA32)
                targetos LINUX
                export BUILD_TARGET_TOOLCHAIN=GNU
                export BUILD_TARGET=IA32
                ;;
            LINUX64)
                targetos LINUX
                export BUILD_TARGET_TOOLCHAIN=GNU
                export BUILD_TARGET=IA64
                ;;
            IA64)
                targetos LINUX
                export BUILD_TARGET_TOOLCHAIN=GNU
                export BUILD_TARGET=IA64
                ;;
	    X86_64)
	        targetos LINUX
	        export BUILD_TARGET_TOOLCHAIN=GNU
	        export BUILD_TARGET=X86_64
		;;
	    PPC64)
	        targetos LINUX
	        export BUILD_TARGET_TOOLCHAIN=GNU
	        export BUILD_TARGET=PPC64
		;;
	    EM64T)
		targetos LINUX
		export BUILD_TARGET_TOOLCHAIN=GNU
		export BUILD_TARGET=EM64T
	    ;;
            *)		    
                if [ "$1" != "" ]; then
                   printf "Unknown target $1\n"
                fi
                printf "Usage: target ia32|ia64|x86_64|em64t|mips|atom|ppc64\n\n"
                return
                ;;    
            esac
        fi
    
		set_os_identifier
        echo "Build target set to $BUILD_TARGET $message"
    
    	if isAllEmb; then
    	   if [ -e "$TL_DIR/.defaultCT" ]; then
    	      export CARD_TYPE=`cat $TL_DIR/.defaultCT 2>&1 /dev/null`
    	      export PROJ_FILE_DIR=$CARD_TYPE"_Firmware"
    	      echo "Card type set to $CARD_TYPE"
    	   fi
    	else
    	   resetct > /dev/null
    	fi
        
        if [[ "$BUILD_TARGET_OS" != "LINUX" && -e "$TL_DIR/.defaultBSP" ]]; then
    	    export TARGET_BSP=`cat $TL_DIR/.defaultBSP 2>&1 /dev/null`
            export TARGET_BSP_UPCASE=`echo $TARGET_BSP | tr '[:lower:]' '[:upper:]'`
    	    echo "Target BSP is ${TARGET_BSP:-empty.}"
        fi
    
    	if [[ -e "$TL_DIR/.defaultBuildConfig" ]]; then
        	export BUILD_CONFIG=`cat $TL_DIR/.defaultBuildConfig 2>&1 /dev/null`
        	echo "Build Configuration is ${BUILD_CONFIG:-empty.}"
    	fi        
    fi       
    
} # end function

function setrel()
{
	export BUILD_CONFIG=release
	if [ "$TL_DIR" != "" ]; then
		echo $BUILD_CONFIG > $TL_DIR/.defaultBuildConfig
	fi
}

function setdbg()
{
	export BUILD_CONFIG=debug
	if [ "$TL_DIR" != "" ]; then
		echo $BUILD_CONFIG > $TL_DIR/.defaultBuildConfig
	fi
}

setulps()
{
    export BUILD_ULPS="${1:-all}"
    if [ "$TL_DIR" != "" ]; then
         echo $BUILD_ULPS > $TL_DIR/.defaultBuildUlps
    fi
}

setskipulps()
{
    export BUILD_SKIP_ULPS="${1:-none}"
    if [ "$TL_DIR" != "" ]; then
        echo $BUILD_SKIP_ULPS > $TL_DIR/.defaultBuildSkipUlps
    fi
}


settools()
{
    local choices
    local x=0

    if [ "$TL_DIR" == "" -o "$BUILD_TARGET_OS" == "" -o "$BUILD_TARGET" == "" ]; then
      echo "Please use the target command to enable the build environment."                   
      return
    fi

    if [ "$1" != "" ]; then
       export BUILD_TARGET_TOOLCHAIN=$1
    else
       choices=`for x in $TL_DIR/Makerules/Target.*; do echo ${x##*/} | cut -d. -f3; done | sort | uniq`    
       if [ "$choices" != "" ]; then
         for choice in $choices
         do
           echo "$(( x += 1 )) ) $choice"
         done
         printf "choice: "
         read the_selection
	 x=0
         for choice in $choices
         do
	   if [ "$the_selection" == "$(( x += 1 ))" ]; then
             export BUILD_TARGET_TOOLCHAIN="$choice"
	     echo "Setting target tool chain to $choice."
	     return
	   fi
         done
	 echo "Unknown choice."
       else
         echo "No choices available."
       fi
    fi
} #end function

#---------------------------------------------------------------------------
# Function:	getosversions
# In Script:	funcs-ext.sh
# Arguments:	none
# Description:	returns relevant os version
# Uses:
#	Control the version string for all builds.
#---------------------------------------------------------------------------
function getosversions()
{
        if   [ `getBuildPlatform` == "Darwin" ]; then
	    tempchoices=`ls -1 $TL_DIR/Makerules/$BUILD_TARGET_OS/$BUILD_TARGET.$BUILD_TARGET_OS_VENDOR.*.h|xargs -n 1 basename|cut -d"." -f2-|xargs -n 1 -J % basename % .h 2> /dev/null`
        else
	    tempchoices=`ls --indicator-style=none /lib/modules`
        fi
	rval=""
        if [ "$BUILD_TARGET_OS" == "LINUX" ]; then
		for rel in $tempchoices
		do
			if [ -d /lib/modules/$rel/build ]
			then
				rval="$rval ${BUILD_TARGET_OS_VENDOR}.${rel}"
			fi
		done
        else
           rval="$tempchoices"
        fi
	echo "$rval ${BUILD_TARGET_OS_VENDOR}.NONE"
}

#---------------------------------------------------------------------------
# Function:	computebuild26
# In Script:	funcs-ext.sh
# Arguments:	None
# Description:	Set BUILD_26 for LINUX
# Uses: BUILD_TARGET_OS, BUILD_TARGET_OS_VERSION
function computebuild26()
{
    if [ "$BUILD_TARGET_OS" == "LINUX" ]
    then
        if echo $BUILD_TARGET_OS_VERSION|grep -q '^2\.6'
        then
            export BUILD_26=1
        elif echo $BUILD_TARGET_OS_VERSION | grep -q '^3.'
        then
            export BUILD_26=1
        else
            unset BUILD_26
        fi
    fi
}

#---------------------------------------------------------------------------
# Function:	setver
# In Script:	funcs-ext.sh
# Arguments:	$1 [optional], set the os version at the command line.
# Description:	Set the BUILD_TARGET_OS_VERSION for the os, and arch.
# Uses:
#	Control the version string for all builds.
#---------------------------------------------------------------------------
function setver()
{
    local x=0
    local vendor
    local version
    if [ "$TL_DIR" == "" -o "$BUILD_TARGET_OS" == "" -o "$BUILD_TARGET" == "" ]; then
      echo "Please use the target command to enable the build environment."                   
      return
    fi
    if [ "$1" != "" ]; then
       export BUILD_TARGET_OS_VENDOR=$1
       export BUILD_TARGET_OS_VERSION=$2
       computebuild26
	   set_os_identifier
       echo "Setting os version to $BUILD_TARGET_OS_VENDOR $BUILD_TARGET_OS_VERSION (26=${BUILD_26:-0})."
    else
       if [ "$TL_DIR" == "" -o "$BUILD_TARGET_OS" == "" -o "$BUILD_TARGET" == "" ]; then
          echo "Please use the target command to enable the build environment."                   
          return
       fi
       choices=`getosversions`
       if [ "$choices" != "" ]; then
         for choice in $choices
         do
	   vendor=`echo $choice|cut -d"." -f1`
	   version=`echo $choice|cut -d"." -f2-`
           echo "$(( x += 1 )) ) $vendor $version"
         done
         printf "choice: "
         read the_selection
	 x=0
         for choice in $choices
         do	  
	   if [ "$the_selection" == "$(( x += 1 ))" ]; then
	     vendor=`echo $choice|cut -d"." -f1`
	     version=`echo $choice|cut -d"." -f2-`
	     export BUILD_TARGET_OS_VENDOR="$vendor"
             export BUILD_TARGET_OS_VERSION="$version"                         
             computebuild26
			 set_os_identifier
	     echo "Setting os version to $BUILD_TARGET_OS_VENDOR $BUILD_TARGET_OS_VERSION (26=${BUILD_26:-0})."
	     return
	   fi
         done
	 echo "Unknown choice."
       else
         echo "No version choices available."
       fi
    fi
} #end function

##
## getIntelcompilervarsfile
##
## Parameters:
##   prefix - directory pattern to search for in /opt/intel
##   subdir - subdirectory under the pattern - can be null
## 
## Looks for a list of directories matching the pattern and selects the newest
##

getIntelcompilervarsfile()
{
	prefix=$1
	subdir=$2
	foundfile=
	for f in $(ls -rd /opt/intel/${prefix}*)
	do
		if [ -f $f/$subdir/bin/compilervars.sh ]
		then
			foundfile=$f/$subdir/bin/compilervars.sh
			break
		fi
	done
	echo "$foundfile"
}
