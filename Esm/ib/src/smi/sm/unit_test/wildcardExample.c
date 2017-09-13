/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015, Intel Corporation

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */
// Unit test program to test wildcarding in the node description
// Various debugging and timing can be turned on/off with #defines below
// Compile:  gcc wildcardExample.c -ltr


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <regex.h>
#include <time.h>

#define DEBUG_ON
//#define TIMING

typedef int bool;
#define true 1
#define false 0
#define MAX_STRING_LENGTH 20
#define MAX_BRACKETS_SUPPORTED 5

typedef struct _RegexParseInfo{
  int totalGroups;
  bool portRangeDefined;
  int portNum1;
  int portNum2;
  int lead0sPort1;
  int lead0sPort2;

  int numBracketRangesDefined;
  int number1[MAX_BRACKETS_SUPPORTED];
  int numDigitsNum1[MAX_BRACKETS_SUPPORTED];  
  int number2[MAX_BRACKETS_SUPPORTED];
  int numDigitsNum2[MAX_BRACKETS_SUPPORTED]; 
  int leading0sNum1[MAX_BRACKETS_SUPPORTED];
  int leading0sNum2[MAX_BRACKETS_SUPPORTED];
  int bracketGroupNum[MAX_BRACKETS_SUPPORTED];
} RegexParseInfo_t;


bool convertWildcardedString(char* inStr, char* outStr, RegexParseInfo_t* regexInfo);
bool processBracketSyntax(char* inputStr, int inputLen, int* curIdxPtr, RegexParseInfo_t* regexInfo, bool isPortRange);
bool isCharValid(char inChar);
void clearRegexStruct(RegexParseInfo_t* regexInfo);
void printRegexStruct(RegexParseInfo_t* regexInfo);
bool isNumberInRange(char* number, RegexParseInfo_t* regexInfo, int bracketIdx);
void populateExpressionsToTest(char* expr[]);
void populateNodeNames(char* nodeNamesArray[]);

int main()
{
#ifdef TIMING
  #ifdef DEBUG_ON
  printf("SHOULDN'T RUN TIMING WITH DEBUG TURNED ON\n");
  exit(1);
  #endif
#endif

  //Populate node names and expressions to test
  char* expr[50];
  populateExpressionsToTest(&expr[0]);
  char* nodeNamesArray[55];
  populateNodeNames(&nodeNamesArray[0]);

  //initialize variables
  bool isMatch = false;
  size_t maxGroups = 10;
  regmatch_t groupArray[maxGroups];
  RegexParseInfo_t regexInfo;

#ifdef DEBUG_ON
  printf("Node Names in Sample Application used for Node Description matching\n");
  int tempIdx;
  for (tempIdx=0; tempIdx<50; tempIdx++) {
    printf("%s\n", nodeNamesArray[tempIdx]);
  }
#endif

  //
  // Loop on all node description expressions to evaluate against all node names
  //

  int exprIdx;
  for (exprIdx=0; exprIdx<50; exprIdx++) {
  //for (exprIdx=0; exprIdx<1; exprIdx++) {

    //variables needed per regular expression we test
    char regexString[100] = "";
    bool isSyntaxValid = false;

    //clear regexInfo from previous expression
    clearRegexStruct(&regexInfo);

    isSyntaxValid = convertWildcardedString(expr[exprIdx], &regexString[0], &regexInfo);

#ifdef DEBUG_ON
    printf("\nProcessing nodeDescription = %s\n", expr[exprIdx]); //regular expression:  %s, &regexString[0]
    //printf("\nProcessing nodeDescription = %s   regular expression:  %s\n", expr[exprIdx], &regexString[0]);
#endif

    //compile into regular expression if there aren't any syntax issues
    if (isSyntaxValid) {

      regex_t regexCompiled;
      if (regcomp(&regexCompiled, &regexString[0], REG_EXTENDED)) {
	printf("Could not compile regular expression  %s\n", &regexString[0]);
	return false;
      }


      //
      // Evaluate all nodes
      //

#ifdef TIMING
      struct timespec time1,time2;
      clock_gettime(CLOCK_REALTIME, &time1);

      int extraLoopIdx;
      for (extraLoopIdx=0; extraLoopIdx<10000; extraLoopIdx++) { //simulate 500k nodes
	//for (extraLoopIdx=0; extraLoopIdx<200000; extraLoopIdx++) { //simulate 10M nodes
#endif

      int nodeIdx;
      for (nodeIdx=0; nodeIdx<55; nodeIdx++) {
	//for (nodeIdx=0; nodeIdx<1; nodeIdx++) {

	if (regexec(&regexCompiled, nodeNamesArray[nodeIdx], maxGroups, groupArray, 0) == 0)
    	  isMatch = true;
	else
	  isMatch = false;


	if (isMatch) {

	  //evaluate all the bracket ranges defined
	  int bracketIdx;
	  for (bracketIdx=0; bracketIdx<regexInfo.numBracketRangesDefined;bracketIdx++) {

	    int bracketGroupNum = regexInfo.bracketGroupNum[bracketIdx];

	    //verify the string index of the group we are evaluting isn't -1 
	    if (groupArray[bracketGroupNum].rm_so != (size_t)-1) {

	      //Make a copy of the node name, which we will manipulate to extract the number out of using the regmatch_t struct fields
	      char nodeName[strlen(nodeNamesArray[nodeIdx]) + 1];
	      cs_strlcpy(nodeName, nodeNamesArray[nodeIdx], sizeof(nodeName));
	      nodeName[groupArray[bracketGroupNum].rm_eo] = 0;

	      //Get the number to evalute by incrementing the node name to the position of the number
	      char* numberToEval = nodeName + groupArray[bracketGroupNum].rm_so;

	      isMatch = isNumberInRange(numberToEval, &regexInfo, bracketIdx);

	      if (!isMatch)
		break;
	    }
	    else {
	      printf("ERROR: Invalid group number for bracket evaluation\n");
	      isMatch = false;
	    }

	  } //loop on all bracket ranges defined

	} //end isMatch check

#ifdef DEBUG_ON
	if (isMatch)
	  printf("MATCH:      %s\n", nodeNamesArray[nodeIdx]);
#endif

	//TODO - if match and port number range defined, need to use port numbers to set bits (will do when we fold into the SM)

      } //end loop on all node names


#ifdef TIMING
      } //end additional loop

      clock_gettime(CLOCK_REALTIME, &time2);

      struct timespec diff;
      if ( (time2.tv_nsec-time1.tv_nsec) < 0) {
	diff.tv_sec = (float)(time2.tv_sec-time1.tv_sec-1);
	diff.tv_nsec = (float)(1000000000+time2.tv_nsec-time1.tv_nsec);
      }
      else {
	diff.tv_sec = (float)(time2.tv_sec-time1.tv_sec);
	diff.tv_nsec = (float)(time2.tv_nsec-time1.tv_nsec);
      }
      printf("ExprIdx: %d   dif %f : %f sec:micro)\n", exprIdx, (float)diff.tv_sec, (float)diff.tv_nsec/1000);
#endif

      regfree(&regexCompiled);
    } //end isSyntaxValid check

#ifdef TIMING
    else {
      printf("ExprIdx %d INVALID\n", exprIdx);
    }
#endif

#ifdef DEBUG_ON
    else {
      printf("Node description syntax INVALID\n");
    }
#endif

  } //end for loop on expressions

  return 0;
}

bool convertWildcardedString(char* inStr, char* outStr, RegexParseInfo_t* regexInfo)
{
  //Convert nodeDesc into regular expression syntax
  //if nodeDesc doesn't start with *, insert ^ at front
  //if nodeDesc doesn't end with *, insert $ at the end
  char* starWildCard = "(([_,=.a-z0-9A-Z[:space:]-])*)";
  int lengthStarWildCard = strlen(starWildCard);
  char* quesWildCard = "(([_,=.a-z0-9A-Z[:space:]-])?)";
  int lengthQuesWildCard = strlen(quesWildCard);
  char* bracketSyntax = "([0-9]+)";
  int lengthBracketSyntax = strlen(bracketSyntax);

  bool isSyntaxValid = true;
  int inStrIdx = 0;
  int outStrIdx = 0;
  int inputLen = strlen(inStr);
  bool needToEvaluateNumbersInReturn = false;

  bool firstBracketValid = true;
  bool firstTimeThroughLoop = true;

  while (inStrIdx<inputLen) {

    //Enforce rules for first character
    if (inStrIdx == 0) {

      if (inStr[inStrIdx] == '?') {

	//walk buffer to find next non ?..if *, insert ^ (?s will be ignored below when we process the characters)
	bool breakLoop=false;
	int curIdx = inStrIdx+1;
	while (!breakLoop) {

	  if (curIdx < inputLen) {

	    if (inStr[curIdx] != '?') {

	      if (inStr[inStrIdx] != '*') { 
		//append ^ character
		strcat(&outStr[outStrIdx], "^");
		outStrIdx++;
	      }
	      breakLoop=true;
	    }
	    else
	      curIdx++;
	  }
	  else
	    breakLoop=true;
	}

      }
      else if (inStr[inStrIdx] != '*') {
	//if first character not '*', append ^ character
	strcat(&outStr[outStrIdx], "^");
	outStrIdx++;
      }

      if (inStr[inStrIdx] == '[') {
	//if first character '[', make sure there is another valid character after the ']'
	//set firstBracketValid to false until we process another valid character
	firstBracketValid = false;
      }
      
    }

    //if not the first time through loop, and we process a valid character, set the firstBracketValid flag to true
    if ( (!firstTimeThroughLoop) && 
	 ((inStr[inStrIdx] != '[') || (inStr[inStrIdx] != ':')) )
      firstBracketValid = true;


    //Process characters
    if (inStr[inStrIdx] == '*') {

      //append starWildCard to outStr
      strcat(&outStr[outStrIdx], starWildCard);
      outStrIdx += lengthStarWildCard;
      regexInfo->totalGroups+=2;

      //look ahead till next non wildcard value.... ignore all '*' or '?'...if next non wildcard is '[', syntax error
      bool breakLoop=false;
      int curIdx = inStrIdx+1;
      while (!breakLoop) {
	if (curIdx < inputLen) {

	  if ( (inStr[curIdx] == '*') || (inStr[curIdx] == '?') ) {
	    //increment index past this
	    inStrIdx++;
	  }
	  else if (inStr[curIdx] == '[') {
	    isSyntaxValid = false;
	    break;
	  }
	  else
	    breakLoop=true;

	  curIdx++;
	}
	else
	  breakLoop=true;
      }

    }
    else if (inStr[inStrIdx] == '?') {

      bool skipQues = false;

      //look ahead till next non ? value.... ignore if it is a '*'...if next non wildcard is '[', syntax error
      bool breakLoop=false;
      int curIdx = inStrIdx+1;
      while (!breakLoop) {

	if (curIdx < inputLen) {

	  if (inStr[curIdx] != '?') {

	    if (inStr[curIdx] == '*') {
	      //increment index past this ?
	      skipQues = true;
	    }
	    else if (inStr[curIdx] == '[') {
	      isSyntaxValid = false;
	    }

	    breakLoop=true;

	  }
	  else
	    curIdx++;
	}
	else
	  breakLoop=true;
      }
	  
      if (!skipQues) {
	//append quesWildCard to outStr
	strcat(&outStr[outStrIdx], quesWildCard);
	outStrIdx += lengthQuesWildCard;
	regexInfo->totalGroups+=2;
      }

    }
    else if (inStr[inStrIdx] == '[') {

      bool isBracketSyntaxValid = processBracketSyntax(&inStr[0], inputLen, &inStrIdx, regexInfo, false);

      if (isBracketSyntaxValid) {
	//append bracketWildCard to outStr
	strcat(&outStr[outStrIdx], bracketSyntax);
	outStrIdx += lengthBracketSyntax;
      }
      else {
	isSyntaxValid = false;
      }

    }
    else if (inStr[inStrIdx] == ':') {
      //:[##-##] must be last in input sequence
      //skip past the ':'
      inStrIdx++;

      //next character has to be '['.....else, error
      if (inStr[inStrIdx] == '[') {
	  
	isSyntaxValid = processBracketSyntax(&inStr[0], inputLen, &inStrIdx, regexInfo, true);

	//continue processing if syntax is valid
	if (isSyntaxValid) {

	  if (inStrIdx+1 >= inputLen) {
	    //append $ character if [] is last in sequence
	    strcat(&outStr[outStrIdx], "$");
	    outStrIdx++;
	  }
	  else {
	    isSyntaxValid = false;
	  }
	    
	}
      }
      else {
	isSyntaxValid = false;
      }

    }
    //handle '.'
    else if (inStr[inStrIdx] == '.') {
      strcat(&outStr[outStrIdx], "\\");
      outStrIdx++;
      strncpy(&outStr[outStrIdx], &inStr[inStrIdx],1);
      outStrIdx++;
    }
    else { //all other character inputs

      if (isCharValid(inStr[inStrIdx])) {
	  //copy the character from the input string and increment outStrIdx
	  strncpy(&outStr[outStrIdx], &inStr[inStrIdx],1);
	  outStrIdx++;
      }
      else {
	isSyntaxValid = false;
      }

    }

    if (!isSyntaxValid)
      break;

    //Enforce rules for last character
    if (inStrIdx == inputLen-1) { 

      //if not '*', append $
      if (inStr[inStrIdx] != '*') {
	//append $ character
	strcat(&outStr[outStrIdx], "$");
	outStrIdx++;
      }

    }

    //increment input string index
    inStrIdx++;

    firstTimeThroughLoop = false;

  } //end processing input sequence

  //NUL terminate the outStr
  outStr[outStrIdx] = '\0';


  if (!firstBracketValid)
    isSyntaxValid = false;

  return isSyntaxValid;
}


bool isCharValid(char inChar) {

  bool isValid = false;

  if ( (inChar >= '0' && inChar  <= '9') ||
       (inChar >= 'a' && inChar  <= 'z') ||
       (inChar >= 'A' && inChar  <= 'Z') ||
       (inChar == '-') ||
       (inChar == ',') ||
       (inChar == '=') ||
       (inChar == '_') ||
       (inChar == ' ') ||
       (inChar == '.') ) {
    isValid = true;
  }

  return isValid;
}


bool processBracketSyntax(char* inputStr, int inputLen, int* curIdxPtr, RegexParseInfo_t* regexInfo, bool isPortRange) {

  bool isSyntaxValid = true;
  int maxNumDigits = 6; //5 + '\0'

  //increment curIdx past the '['
  int curIdx = *curIdxPtr;
  curIdx++;

  //flow control variables
  bool parsingNum1 = true;
  bool countingLeadingZeros = true;

  //will use these to build the regular expression
  int leadingZeroNum1=0, leadingZeroNum2=0;
  int numDigitsNum1=0, numDigitsNum2=0;
  char* num1charPtr;
  char* num2charPtr;

  //loop to process the rest of the [##-##] syntax
  int tempNumDigits = 0;
  int tempLeading0s = 0;
  while (curIdx<inputLen) {
    //look for "-" character
    if ( (parsingNum1) && (inputStr[curIdx] == '-') ) {

      //end of 1st number, switch to start parsing the 2nd number
      parsingNum1 = false;
      leadingZeroNum1 = tempLeading0s;
      numDigitsNum1 = tempNumDigits;

      //handle a range that starts with 0
      if ( (numDigitsNum1 == 0) && (leadingZeroNum1==1) ) {
	numDigitsNum1 = 1;
	leadingZeroNum1 = 0;

	//set num1charPtr 
	num1charPtr = &inputStr[curIdx-1];
      }

      //transition to start process 2nd number
      tempLeading0s = 0;
      tempNumDigits = 0;
      curIdx++;
      countingLeadingZeros = true;
    }
    else if ( (!parsingNum1) && (inputStr[curIdx] == ']') ) {
      //end of valid bracket expression
      leadingZeroNum2 = tempLeading0s;
      numDigitsNum2 = tempNumDigits;

      //update current index pointer
      *curIdxPtr = curIdx;

      //validate number information to this point
      if ( (numDigitsNum1 <= 0) || 
	   (numDigitsNum2 <= 0) ||
	   (numDigitsNum1>maxNumDigits-1) || 
	   (numDigitsNum2>maxNumDigits-1) ||
	   (numDigitsNum1 > numDigitsNum2) ||
	   ((numDigitsNum1==numDigitsNum2) && (leadingZeroNum1 != leadingZeroNum2)) ) {
	isSyntaxValid = false;
      }

      break;
    }
    else if (inputStr[curIdx] >= '0' &&inputStr[curIdx]  <= '9') {

      //count number of leading 0s
      if ( (inputStr[curIdx] == '0') && (countingLeadingZeros) ) {
	tempLeading0s++;
      }
      else {
	//if first non-zero number
	if (tempNumDigits == 0) {
	  countingLeadingZeros = false;

	  if (parsingNum1)
	    num1charPtr = &inputStr[curIdx];
	  else
	    num2charPtr = &inputStr[curIdx];
	}

	tempNumDigits++;
      }

      curIdx++;

    }
    else {
      //if any other character, input is invalid
      isSyntaxValid = false;
      break;
    }
  } //end loop through input syntax


  //translate numbers to ints and save them off
  if (isSyntaxValid) {

    //convert 1st number
    int size = numDigitsNum1+1;
    char firstNum[maxNumDigits];
    firstNum[size-1] = '\0';
    strncpy(&firstNum[0], num1charPtr, numDigitsNum1);
    int num1 = atoi(&firstNum[0]);

    //convert 2nd number
    size = numDigitsNum2+1;
    char secondNum[maxNumDigits];
    secondNum[size-1] = '\0';
    strncpy(&secondNum[0], num2charPtr, numDigitsNum2);
    int num2 = atoi(&secondNum[0]);

    //validate num2 isn't larger than num1
    if (num1 > num2)
      isSyntaxValid = false;

    //set regexInfo fields
    regexInfo->totalGroups++;

    if (isPortRange) {
      regexInfo->portRangeDefined = true;
      regexInfo->portNum1 = num1;
      regexInfo->portNum2 = num2;
      regexInfo->lead0sPort1 = leadingZeroNum1;
      regexInfo->lead0sPort2 = leadingZeroNum2;
    }
    else {
      regexInfo->numBracketRangesDefined++;

      int idx = regexInfo->numBracketRangesDefined-1;
      regexInfo->number1[idx] = num1;
      regexInfo->numDigitsNum1[idx] = numDigitsNum1;
      regexInfo->number2[idx] = num2;
      regexInfo->numDigitsNum2[idx] = numDigitsNum2;
      regexInfo->leading0sNum1[idx] = leadingZeroNum1;
      regexInfo->leading0sNum2[idx] = leadingZeroNum2;
      regexInfo->bracketGroupNum[idx] = regexInfo->totalGroups;
    }

  }

  return isSyntaxValid;

}


bool isNumberInRange(char* number, RegexParseInfo_t* regexInfo, int bracketIdx) {

  bool isValid = true;
  bool isInRange = false;

  //local/temp and flow control variables
  int numDigits = 0;
  int numberLeading0s = 0;
  int curIdx = 0;
  int inputLen = strlen(number);

  bool countingLeadingZeros = true;
  char* charPtr;

  //convert number from char* to int (count leading 0s)
  while (curIdx<inputLen) {
    
    //verify this index is a valid number
    if (number[curIdx] >= '0' &&number[curIdx]  <= '9') {

      //count number of leading 0s
      if ( (number[curIdx] == '0') && (countingLeadingZeros) )
	numberLeading0s++;
      else {

	if (numDigits == 0) {
	  countingLeadingZeros = false;
  	  charPtr = &number[curIdx];
	}

	numDigits++;
      }

      curIdx++;
    }
    else {
      isValid = false;
      break;
    }
  }

  if (isValid) {
    int maxNumDigits = 6; //5 + '\0'
    char numArray[maxNumDigits];

    int size = numDigits+1;
    numArray[size-1] = '\0';
    strncpy(&numArray[0], charPtr, numDigits);
    int numberToEvalute = atoi(&numArray[0]);

    if ( (regexInfo->leading0sNum1[bracketIdx] == 0) && (regexInfo->leading0sNum1[bracketIdx] == 0) && (numberLeading0s == 0) ) {
      //no leading zeros, just do simple comparison
      if ( (numberToEvalute >= regexInfo->number1[bracketIdx]) && (numberToEvalute <= regexInfo->number2[bracketIdx]) )
	isInRange = true;
    }
    else {
      //evaluate leading 0s
      int numToEvalDigitsDiff = numDigits - regexInfo->numDigitsNum1[bracketIdx];
      int num0sToExpect = regexInfo->leading0sNum1[bracketIdx] - numToEvalDigitsDiff;

      if (num0sToExpect == numberLeading0s) {
	//finish evaluation
	if ( (numberToEvalute >= regexInfo->number1[bracketIdx]) && (numberToEvalute <= regexInfo->number2[bracketIdx]) )
	  isInRange = true;
      }
    }

  }

  return isInRange;
}


void clearRegexStruct(RegexParseInfo_t* regexInfo) {

  regexInfo->totalGroups = 0;

  //clear port range fields
  regexInfo->portRangeDefined = false;
  regexInfo->portNum1 = 0;
  regexInfo->portNum2 = 0;
  regexInfo->lead0sPort1 = 0;
  regexInfo->lead0sPort2 = 0;

  //clear number range bracket fields
  regexInfo->numBracketRangesDefined = 0;

  int idx;
  for (idx=0; idx<MAX_BRACKETS_SUPPORTED; idx++) {
    regexInfo->number1[idx] = 0;
    regexInfo->number2[idx] = 0;
    regexInfo->leading0sNum1[idx] = 0;
    regexInfo->leading0sNum2[idx] = 0;
    regexInfo->bracketGroupNum[idx] = 0;
  }
}

void printRegexStruct(RegexParseInfo_t* regexInfo) {

  if ( (regexInfo->portRangeDefined == false) && (regexInfo->numBracketRangesDefined == 0) ) {
    printf("Regex Info Struct: No port ranges or brackets defined\n");
  }
  else {
    printf("Regex Info Struct:\n");

    int idx;
    for (idx=0; idx<regexInfo->numBracketRangesDefined; idx++) {
      printf("Bracket Range Info %d:\n", idx);
      printf("             Num1: %d\n", regexInfo->number1[idx]);
      printf("             Num2: %d\n", regexInfo->number2[idx]);
      printf("    Leading0sNum1: %d\n", regexInfo->leading0sNum1[idx]);
      printf("    Leading0sNum1: %d\n", regexInfo->leading0sNum2[idx]);
      printf("         GroupNum: %d\n", regexInfo->bracketGroupNum[idx]);
    }

    if (regexInfo->portRangeDefined == true) {
      printf("Port Range Info:\n");
      printf("         PortNum1: %d\n", regexInfo->portNum1);
      printf("         PortNum2: %d\n", regexInfo->portNum2);
      printf("    Leading0sNum1: %d\n", regexInfo->lead0sPort1);
      printf("    Leading0sNum2: %d\n", regexInfo->lead0sPort2);
    }
  }

}

void populateExpressionsToTest(char* expr[]) {
  expr[0] = "switch?";
  expr[1] = "switch*";
  expr[2] = "*switch?";
  expr[3] = "?switch*";
  expr[4] = "switch*1";
  expr[5] = "node-name[00124-01202]";
  expr[6] = "node-name[0218-3265]";
  expr[7] = "node-name[0218-0326]";
  expr[8] = "node-name[0218-3002]";
  expr[9] = "node-name[002-042]";
  expr[10] = "node[2-4]-clustA";
  expr[11] = "switch1:[1-48]";
  expr[12] = "switch:[01-24]";
  expr[13] = "switch*:[1-24]";
  expr[14] = "*switch?:[4-5]";
  expr[15] = "node-name[0218-3002]:[1-3]";
  expr[16] = "chassis[1-5]-node[001-100]:[1-48]";
  expr[17] = "?swi*h[3-20]";
  expr[18] = "[1-10]switch";
  expr[19] = "switch?1";
  expr[20] = "switch*?1";
  expr[21] = "switch??";
  expr[22] = "chassis[01-10]-node[49-51]:[1-48]";
  expr[23] = "node.chassis[01-99]";
  expr[24] = "switch-[1-20]";
  expr[25] = "switch?*"; //ignore ?
  expr[26] = "switch*?"; //ignore ?
  expr[27] = "switch**"; //should produce same as switch*
  expr[28] = "switch**?"; //should produce same as switch*
  expr[29] = "?*switch"; //should produce same as *switch

  //error cases
  expr[30] = "(switch?";
  expr[31] = ")switch?";
  expr[32] = "{switch?";
  expr[33] = "}switch?";
  expr[34] = "$switch?";
  expr[35] = "+switch?";
  expr[36] = "|switch?";
  expr[37] = "node[413-200]";
  expr[38] = "node[0200-400]";
  expr[39] = "node[ab-401]";
  expr[40] = "[10-20]";
  expr[41] = "node[a";
  expr[42] = "[]";
  expr[43] = "nodeName[]"; 
  expr[44] = "^switch?";
  expr[45] = "switch*?[1-15]";
  expr[46] = "switch*[3-20]";
  expr[47] = "node*[1-24]";
  expr[48] = "\\switch?";
  expr[49] = "/switch?";

}

void populateNodeNames(char* nodeNamesArray[]) {
  nodeNamesArray[0] = "switch1";
  nodeNamesArray[1] = "switch";
  nodeNamesArray[2] = "switch11";
  nodeNamesArray[3] = "switch20";
  nodeNamesArray[4] = "9switch1";
  nodeNamesArray[5] = "switchabc1";
  nodeNamesArray[6] = "switchdef10";
  nodeNamesArray[7] = "switchhij11";
  nodeNamesArray[8] = "xyzswitch2";
  nodeNamesArray[9] = "a2bswitch";
  nodeNamesArray[10] = "node-name0115";
  nodeNamesArray[11] = "node-name125";
  nodeNamesArray[12] = "node-name299";
  nodeNamesArray[13] = "node-name324";
  nodeNamesArray[14] = "node-name0218";
  nodeNamesArray[15] = "node-name0324";
  nodeNamesArray[16] = "node-name3001";
  nodeNamesArray[17] = "node-name0217";
  nodeNamesArray[18] = "node-name00218";
  nodeNamesArray[19] = "node-name00324";
  nodeNamesArray[20] = "node-name01201";
  nodeNamesArray[21] = "node-name002";
  nodeNamesArray[22] = "node-name031";
  nodeNamesArray[23] = "node-name01";
  nodeNamesArray[24] = "node-name02";
  nodeNamesArray[25] = "node-name040";
  nodeNamesArray[26] = "node-name042";
  nodeNamesArray[27] = "node1-clustA";
  nodeNamesArray[28] = "node2-clustA";
  nodeNamesArray[29] = "node3-clustA";
  nodeNamesArray[30] = "node4-clustA";
  nodeNamesArray[31] = "node5.clustA";
  nodeNamesArray[32] = "chassis3-node050";
  nodeNamesArray[33] = "chassis02-node50";
  nodeNamesArray[34] = "chassis003-node50";
  nodeNamesArray[35] = "chassis5-node050";
  nodeNamesArray[36] = "chassis05-node50";
  nodeNamesArray[37] = "5switch";
  nodeNamesArray[38] = "node.chassis5";
  nodeNamesArray[39] = "nodeKchassis06";
  nodeNamesArray[40] = "node.chassis99";
  nodeNamesArray[41] = "switch-11";
  nodeNamesArray[42] = "switch-01";
  nodeNamesArray[43] = "node001-clustA";
  nodeNamesArray[44] = "node02-clustA";
  nodeNamesArray[45] = "node003-clustA";
  nodeNamesArray[46] = "node04-clustA";
  nodeNamesArray[47] = "node05.clustA";
  nodeNamesArray[48] = "chassis3-node5";
  nodeNamesArray[49] = "chassis7-node50";
  nodeNamesArray[50] = "switch ";
  nodeNamesArray[51] = "switch  ";
  nodeNamesArray[52] = "    switch ";
  nodeNamesArray[53] = " switch  1";
  nodeNamesArray[54] = "switch 1";
}
