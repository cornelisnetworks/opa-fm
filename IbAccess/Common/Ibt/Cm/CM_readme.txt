IBA SDK3 Communication Manager(CM) Component Release Notes
____________________________________________________________________



A) VERSION NOTES
____________________________________________________________________
SDK3


B) FEATURES
____________________________________________________________________
The CM component is implemented according to IB spec 1.0A.
It supports both client/server and peer-to-peer connection
models. It also support Service ID Resolution (SIDR) protocol.
The CM can be accessed via user-mode or kernel-mode.
The CM supports all required messages (REQ, REP, REJ..) defined in IB spec
including SIDR messages. The optional LAP and APR messages are not supported.

C) KNOWN ISSUES
____________________________________________________________________
VIPL has reported seeing connection callback delays on Saber-RX system. Please
consult VIPL notes to workaround this issue.

VIPL has reported seeing CmCreateCEP() fail on multiple adapters environment.
There is no resolution at this point other than a reboot.

SIDR Query timing out. Increase the SidrReqTimeoutMs registry value.


D) GENERAL NOTES
____________________________________________________________________
The CM component is part of the IBT driver stack. The CM can be accessed directly via the CM API.


E) INSTALLATION NOTES
____________________________________________________________________
The CM component is installed as part of the IBT driver stack. No additional
installation is required.
