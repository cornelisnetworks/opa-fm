#!/usr/bin/env python2
# coding=utf-8
#
# BEGIN_ICS_COPYRIGHT9
#
# INTEL CONFIDENTIAL
# Copyright 2018 Intel Corporation All Rights Reserved.
#
# The source code contained or described herein and all documents related to the
# source code ("Material") are owned by Intel Corporation or its suppliers or
# licensors. Title to the Material remains with Intel Corporation or its
# suppliers and licensors. The Material may contain trade secrets and
# proprietary and confidential information of Intel Corporation and its
# suppliers and licensors, and is protected by worldwide copyright and trade
# secret laws and treaty provisions. No part of the Material may be used,
# copied, reproduced, modified, published, uploaded, posted, transmitted,
# distributed, or disclosed in any way without Intel's prior express written
# permission.
#
# No license under any patent, copyright, trade secret or other intellectual
# property right is granted to or conferred upon you by disclosure or delivery
# of the Materials, either expressly, by implication, inducement, estoppel or
# otherwise. Any license under such intellectual property rights must be
# express and approved by Intel in writing.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Unless otherwise agreed by Intel in writing, you may not remove or alter this
# notice or any other notice embedded in Materials by Intel or Intel's suppliers
# or licensors in any way.
#
# END_ICS_COPYRIGHT9
#
# Author: Peter Littlefield (peter.littlefield@intel.com)

import re
import sys
#import string
import argparse
from operator import attrgetter

def print_err(*args, **kwargs):
    """Print to stderr."""
    sys.stderr.write(*args, **kwargs)
    sys.stderr.write("\n")
    sys.stderr.flush()

class ReleaseTag(dict):
    """Generic class used to store and convert release tags to different formats."""

#    GroupRegex = re.compile(r"^[_\.]?(?P<Field>[MPIABRC]*)(?P<Version>[0-9a-z]+)(?P<Remainder>.*)")
    GroupRegex = re.compile(r"^[_\.]?(?P<Alphas>[A-Z]*)(?P<Numerals>[0-9]+)(?P<Remainder>.*)")
#    FieldKeys = ["X", "Y", "M", "P", "I"]
#    RpmTrans = string.maketrans("ABRC", "abrc")

    # Dicts defining the format for binary/hex-encoded release tags.
    # FieldName: (Mask, Shift)
    HexFieldInfo32bit = { "Major": (0xF, 28), "Minor": (0xFF, 20), "Maintenance": (0xFF, 16), "EncodedQuality32bit": (0xF, 12), "BuildNumber": (0xFFF, 0) }
    HexFieldInfo64bit = { "Major": (0xFF, 56), "Minor": (0xFF, 48), "Maintenance": (0xFF, 40), "EncodedQuality64bit": (0xF, 36), "Patch": (0xF, 32), "BuildNumber": (0xFFFFFFFF, 0) }

    def __init__(self, **kwargs):
        # Call "dict" constructor with default value of zero for all FieldKeys.
        super(ReleaseTag, self).__init__([(FieldKey, 0) for FieldKey in self.FieldKeys])
        # Update with values passed in via kwargs.
        self.update(kwargs)

    @classmethod
    def FromTagString(cls, TagString):
        """Factory method to construct a ReleaseTag object based on the supplied TagString.

        This will construct the appropriate subclass based on the input format.  If the supplied TagString does not match
        a valid format, returns None."""

        # Exclude any characters before the first numeral.
        RawTag = TagString[re.search("[0-9]", TagString).start():]

        if re.search(r"[^ABPRC0-9_\.]", RawTag):
            # Check for any characters that are not supported in the new Alphanumeric tag format (e.g. lowercase letters, uppercase letters other than A, B, P, RC.)
            # TODO: This could probably be handled more cleanly in the main "GroupRegex".
            return ReleaseTagLegacy(RawTag)

        # Break the input TagString into pairs consisting of zero or more alpha characters, followed by 1 or more numerals.
        Remainder = RawTag
        AlphaNumeralPairs = []
        while Remainder:
            try:
                Alphas,Numerals,Remainder = cls.GroupRegex.match(Remainder).group("Alphas", "Numerals", "Remainder")
            except AttributeError:
                raise(ValueError("Invalid Tag Format: " + TagString + " Cannot match: " + Remainder))
            AlphaNumeralPairs.append( (Alphas,Numerals) )

        #print(cls.__subclasses__())
        # Pass the list of Alpha-Numeral pairs to each subclass's "FromAlphaNumeralPairs" factory method.
        ConstructedReleaseTags = [subclass.FromAlphaNumeralPairs(AlphaNumeralPairs) for subclass in cls.__subclasses__()]
        #print("ReleaseTag.FromTagString(): ConstructedReleaseTags: " + str(ConstructedReleaseTags))
        ConstructedReleaseTags = [ConstructedReleaseTag for ConstructedReleaseTag in ConstructedReleaseTags if ConstructedReleaseTag]
        if len(ConstructedReleaseTags) > 1:
            print_err("Warning: Multiple objects constructed for Release Tag: " + TagString)

        try:
            return ConstructedReleaseTags.pop(0)
        except IndexError:
            print_err("Fatal: No matching format found for release tag: " + TagString)
            raise(ValueError("Fatal: No matching format found for release tag: " + TagString))

    def Get32BitHex(self):
        Hex = 0x00000000
        for FieldKey, (Mask, Shift) in self.HexFieldInfo32bit.iteritems():
            if self[FieldKey] > Mask:
                print_err("Warning: \"" + FieldKey + "\": " + str(self[FieldKey]) + ": Out of range for binary format. Clamping to: " + str(Mask))
            Hex |= (min(self[FieldKey], Mask) & Mask) << Shift
        #return Hex
        return "0x%08X"%Hex

    def Get64BitHex(self):
        Hex = 0x0000000000000000
        for FieldKey, (Mask, Shift) in self.HexFieldInfo64bit.iteritems():
            if self[FieldKey] > Mask:
                print_err("Warning: \"" + FieldKey + "\": " + str(self[FieldKey]) + ": Out of range for binary format. Clamping to: " + str(Mask))
            Hex |= (min(self[FieldKey], Mask) & Mask) << Shift

        #return Hex
        return "0x%016X"%Hex

class ReleaseTagLegacy(object):
    """Class used to represent release tags containing lower-case letters or other otherwise-unsupported characters.

    This allows support for developer-level build release tags which often contain a username, such as those produced by "MakeTools/patch_engineer_version.sh".
    e.g. "0rootopa0" in FM_SMOKE_TEST."""
    # TODO: We could reimplement the parsing algorithm in "patch_version"/"convert_releasetag.pl" (with special handling of "P", "M", and "I" characters).
    # In this case we'd probably want to make an actual subclass of ReleaseTag and correctly populate the fields.
    def __init__(self, RawTag):
        """Constructor. Simply replaces underscores with dots."""
        self.VersionString = RawTag.replace("_", ".")

    def GetText(self):
        return self.VersionString

    def GetRpmName(self):
        return self.GetText()

    def GetBriefRpmName(self):
        return self.GetRpmName()

    def Get32BitHex(self):
        # No way to map unsupported characters into hex format, so default to all zeros.
        return "0x00000000"

    def Get64BitHex(self):
        # No way to map unsupported characters into hex format, so default to all zeros.
        return "0x0000000000000000"

    def GetDebianName(self):
        return self.GetText()

class ReleaseTagPV(ReleaseTag):
    """Class to represent a PV-quality release tag. Contains no letters, and "Patch" field is 0 (e.g. "10.8.0.0.100)."""

    FieldKeys = ["Major", "Minor", "Maintenance", "Patch", "BuildNumber"]

    def __init__(self, **kwargs):
        super(ReleaseTagPV, self).__init__(**kwargs)
        self["EncodedQuality32bit"] = 14
        self["EncodedQuality64bit"] = 14

    @classmethod
    def FromAlphaNumeralPairs(cls, AlphaNumeralPairs):
        FieldKeys = list(cls.FieldKeys) # Copy FieldKeys so we can pop elements without side effects.
        Fields = {}

        for Alphas,Numerals in AlphaNumeralPairs:
            # Iterate through pairs of Alpha and Numeral strings
            try:
                FieldKey = FieldKeys.pop(0)
                #print("ReleaseTagQuality.FromAlphaNumeralPairs(): Alphas: " + Alphas + " Numerals: " + Numerals + " FieldKey: " + FieldKey)
            except IndexError:
                # Too many fields in Release Tag for this type.
                return None
            if Alphas:
                # No alphas allowed in this format type.
                return None

            Fields[FieldKey] = int(Numerals)

        if "Patch" in Fields and Fields["Patch"] != 0:
            # If "Patch" field is nonzero, construct a "ReleaseTagPatch" object instead of "ReleaseTagPV".
            return ReleaseTagPatch(**Fields)
        else:
            return cls(**Fields)

    def GetText(self):
        return ".".join([str(self["Major"]), str(self["Minor"]), str(self["Maintenance"]), str(self["Patch"]), str(self["BuildNumber"])])

    def GetRpmName(self):
        RpmName = ".".join([str(self["Major"]), str(self["Minor"]), str(self["Maintenance"]), str(self["Patch"])])
        RpmName += "-" + str(self["BuildNumber"])
        return RpmName

    def GetBriefRpmName(self):
        if self["Patch"] != 0:
            return self.GetRpmName()
        BriefRpmName = ".".join([str(self["Major"]), str(self["Minor"])])
        if self["Maintenance"] != 0:
            BriefRpmName += "." + str(self["Maintenance"])
        BriefRpmName += "-" + str(self["BuildNumber"])
        return BriefRpmName

    def GetDebianName(self):
        # Debian string is the same as Text for formats with no "Quality" field.
        return self.GetText()


class ReleaseTagPatch(ReleaseTagPV):
    """Represents a release tag with nonzero "Patch" field "(e.g. 10.8.0.1.1")"""

    # Override 32-bit hex encoding to include "Patch" field.
    HexFieldInfo32bit = { "Major": (0xF, 28), "Minor": (0xFF, 20), "Maintenance": (0xFF, 16), "EncodedQuality32bit": (0xF, 12), "Patch": (0xF, 8), "BuildNumber": (0xFF, 0) }

    def __init__(self, **kwargs):
        super(ReleaseTagPatch, self).__init__(**kwargs)
        # Override 32-bit encoding of Quality field.
        self["EncodedQuality32bit"] = 15


class ReleaseTagQuality(ReleaseTag):
    """A generic release tag type that contains a "quality" code for Alpha/Beta/Poweron/ReleaseCandidate (e.g. 10.8.0RC1.100, 10.9.1A2.10)."""

    FieldKeys = ["Major", "Minor", "Maintenance", "QualityLevel", "BuildNumber"]

    def __init__(self, **kwargs):
        super(ReleaseTagQuality, self).__init__(**kwargs)
        self["Patch"] = 0
        EncodedQuality = self.QualityEncodeMin + self["QualityLevel"]
        if EncodedQuality > self.QualityEncodeMax:
            print_err("Warning: Encoded Quality out of range: " + str(EncodedQuality) + " Clamping to: " + str(self.QualityEncodeMax))
            # Clamp to max allowed values
            EncodedQuality = self.QualityEncodeMax
        self["EncodedQuality32bit"] = EncodedQuality
        self["EncodedQuality64bit"] = EncodedQuality

    @classmethod
    def FromAlphaNumeralPairs(cls, AlphaNumeralPairs):
        FieldKeys = list(cls.FieldKeys) # Copy FieldKeys so we can pop elements without side effects.
        Fields = {}
        subclass = None

        #print("ReleaseTagQuality.FromAlphaNumeralPairs(): AlphaNumeralPairs: " + str(AlphaNumeralPairs))

        for Alphas,Numerals in AlphaNumeralPairs:
            # Iterate through pairs of Alpha and Numeral strings
            try:
                FieldKey = FieldKeys.pop(0)
                #print("ReleaseTagQuality.FromAlphaNumeralPairs(): Alphas: " + Alphas + " Numerals: " + Numerals + " FieldKey: " + FieldKey)
            except IndexError:
                # Too many fields in Release Tag for this type.
                return None

            if Alphas:
                # Skip to "QualityLevel" field when alpha characters are encountered.
                while "QualityLevel" in FieldKeys:
                    FieldKey = FieldKeys.pop(0)

                if "QualityLevel" in Fields:
                    # Only allow 1 alpha field in Release Tag
                    print_err("ReleaseTagQuality.FromAlphaNumeralPairs(): Too many alpha fields in release tag.")
                    return None
                # Find subclass matching alpha characters.
                MatchingSubclasses = [subclass for subclass in cls.__subclasses__() if subclass.QualityPrefix == Alphas]
                if len(MatchingSubclasses) > 1:
                    print_err("Warning: Multiple matching subclasses found for field: " + Alphas)
                try:
                    subclass = MatchingSubclasses.pop(0)
                except IndexError:
                    # No matching subclass found for this Release Tag format.
                    print_err("ReleaseTagQuality.FromAlphaNumeralPairs(): Error: No matching subclass for Quality: " + Alphas)
                    return None
            elif FieldKey == "QualityLevel":
                # If no numeric characters in this position, format is invalid for this type of Release Tag.
                #print("ReleaseTagQuality.FromAlphaNumeralPairs(): No alpha characters found")
                return None

            Fields[FieldKey] = int(Numerals)
        #print("ReleaseTagQuality.FromAlphaNumeralPairs(): Fields: " + str(Fields))
        if "QualityLevel" in Fields:
            return subclass(**Fields)
        else:
            # No alpha field found in supplied Release Tag, so format is invalid for this type.
            return None

    def GetText(self):
        Text = ".".join([str(self["Major"]), str(self["Minor"]), str(self["Maintenance"])])
        Text += self.QualityPrefix + str(self["QualityLevel"]) + "." + str(self["BuildNumber"])
        return Text

    def GetRpmName(self):
        RpmName = ".".join([str(self["Major"]), str(self["Minor"]), str(self["Maintenance"]), "0"])
        RpmName += "-" + self.RpmQualityPrefix + str(self["QualityLevel"]) + "." + str(self["BuildNumber"])
        return RpmName

    def GetBriefRpmName(self):
        BriefRpmName = ".".join([str(self["Major"]), str(self["Minor"])])
        if self["Maintenance"] != 0:
            BriefRpmName += "." + str(self["Maintenance"])
        BriefRpmName += "-" + self.RpmQualityPrefix + str(self["QualityLevel"]) + "." + str(self["BuildNumber"])
        return BriefRpmName

    def GetDebianName(self):
        DebianName = ".".join([str(self["Major"]), str(self["Minor"]), str(self["Maintenance"]), "0"])
        DebianName += self.RpmQualityPrefix + str(self["QualityLevel"]) + "." + str(self["BuildNumber"])
        return DebianName

class ReleaseTagPoweron(ReleaseTagQuality):
    QualityPrefix = "P"
    RpmQualityPrefix = "P"
    QualityEncodeMin = 0
    QualityEncodeMax = 2

class ReleaseTagAlpha(ReleaseTagQuality):
    QualityPrefix = "A"
    RpmQualityPrefix = "a"
    QualityEncodeMin = 3
    QualityEncodeMax = 5

class ReleaseTagBeta(ReleaseTagQuality):
    QualityPrefix = "B"
    RpmQualityPrefix = "b"
    QualityEncodeMin = 6
    QualityEncodeMax = 8

class ReleaseTagRC(ReleaseTagQuality):
    QualityPrefix = "RC"
    RpmQualityPrefix = "rc"
    QualityEncodeMin = 9
    QualityEncodeMax = 13


def main():
    ArgParser = argparse.ArgumentParser(description="Parse a release tag and output it in one of several formats.")

    Formats = {"text": attrgetter("GetText"), "rpm": attrgetter("GetRpmName"), "briefrpm": attrgetter("GetBriefRpmName"), "debian": attrgetter("GetDebianName"), "32bit": attrgetter("Get32BitHex"), "64bit": attrgetter("Get64BitHex")}

    # TODO: Add help text for each command-line argument.
    ArgParser.add_argument('--format', action='store', choices=sorted(Formats.keys(), reverse=True), default="text", help="Output format. Default: text")
    ArgParser.add_argument('TagString', help="Release Tag to format e.g. 10_8_0RC2_100")
    # TODO: Debug/loglevel option(s)

    args = ArgParser.parse_args()
    #print("TagString: " + str(args.TagString))
    #print("format: " + str(args.format))
    ReleaseTagObject = ReleaseTag.FromTagString(args.TagString)

    OutputFormat = Formats[args.format](ReleaseTagObject)()
    print(OutputFormat)


# Sentinel to run the main function if this file is directly executed (rather than imported).
if __name__ == "__main__":
    main()
