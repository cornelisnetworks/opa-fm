/* BEGIN_ICS_COPYRIGHT3 ****************************************

Copyright (c) 2015-2017, Intel Corporation

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_IB_CM_H_
#define _IBA_IB_CM_H_

/*
 * IB Communication/Connection Management API
 */

/*
 * Kernel and user-mode user shares the same header file/API
 */

/******************************************************************************
 * CM overview
 *
 * The CM is designed such that applications can control the QP directly
 * This permits the use of alternate verbs APIs, QP re-use, and other
 * optimizations.
 *
 * New Applications should use the following flags (set via iba_cm_modify_cep):
 * CM_FLAG_TIMEWAIT_CALLBACK - provides for IBTA compliance in timewait state
 * CM_FLAG_ASYNC_ACCEPT	- improves multi-CEP handling for listeners
 * CM_FLAG_TURNAROUND_TIME - allows appl to specify its turnaround time
 * CM_FLAG_LISTEN_BACKLOG - allow appl to specify maximum listener pending Q
 *
 * The CM offers 2 interfaces:
 * 1. a direct interface via calls to the iba_cm_* routines
 * 2. a function table interface which is SDK3 compatible
 * new applications should use the direct interface.
 *
 * The CM is inheritly callback based.  Callbacks occur in threads, so an
 * application may perform operations in the callback (including
 * iba_cm_destroy_cep).
 * For applications which may establish more than 1 connection at a time, it
 * is recommended the processing in the callback be limited to basic QP
 * operations and the appropriate Cm call (iba_cm_accept, iba_cm_reject, etc).
 * If callback processing is too lengthy, the remote endpoint may timeout.
 * The TURNAROUND_TIME setting can be used to lengthen the timeouts.
 *
 * Internal to the CM protocol there is an MRA feature.  This feature is hidden
 * from the user and is automatically used by the CM based on the timeouts
 * requested in a message and the TURNAROUND_TIME setting for the receiving CEP.
 * Proper use of the TURNAROUND_TIME setting can prevent unnecessary retries and
 * timeouts by the remote endpoint during connection establishment.
 *
 * All Cm calls are thread safe.
 *
 * When CM_FLAG_ASYNC_ACCEPT is used, all CM calls, except iba_cm_wait, are
 * non-blocking.  If CM_FLAG_ASYNC_ACCEPT is not used, the iba_cm_accept
 * call in a server context (accepting a REQ and sending REP) will block waiting
 * for the RTU.
 *
 * The application must explicitly iba_cm_destroy_cep for any CEPs it creates or
 * which are returned to a listener by iba_cm_accept.  Failure to do so will
 * tie up CM resources until the application exits.  Kernel mode drivers
 * are totally responsible for their own cleanup and must be carefully
 * coded not to leak CEPs.
 *
 * In many of the callbacks a Path Record is provided.  The AckTimeout value
 * given should be used for QP modifications.  The PktLifeTime value supplied
 * is rounded up and may include an extra CA Ack Delay/2 factor.  As such
 * it should not be used for QP timeouts.
 *
 * The CM supports 3 basic types of connections:
 * 1. Client/Server RC/UC/RD connection
 * 2. Peer to Peer (2 Clients) RC/UC/RD connection
 * 3. Client/Server UD QP exchange via IBTA SIDR protocol
 *
 * *****************************************************************************
 * Client Server RC/UC/RD Connections
 * -----------------------------------------------------------------------------
 * The Basic Sequence for connection establishment is as follows:
 * (this assumes ASYNC_ACCEPT and TIMEWAIT_CALLBACK are enabled)
 * Server								Client
 * iba_cm_create_cep					iba_cm_create_cep
 * iba_cm_listen
 * 										iba_cm_prepare_request
 * 										iba_cm_connect
 * FCM_CONNECT_REQUEST callback
 * iba_cm_process_request
 * iba_cm_accept w/ reply data
 * (iba_cm_accept returns new CEP)
 * 										FCM_CONNECT_REPLY callback
 *										iba_cm_process_reply
 * 										iba_cm_accept w/Rtu data
 * 										(may now send on QP)
 * FCM_CONNECT_ESTABLISHED callback
 * iba_cm_prepare_rts
 * (may now send on QP)
 *
 * (disconnect could be initiated by either side, client initiates below):
 *										iba_cm_disconnect w/request data
 * FCM_DISCONNECT_REQUEST callback
 * iba_cm_disconnect w/reply data
 * 
 * 					(timewait delay occurs)
 * FCM_DISCONNECTED callback			FCM_DISCONNECTED callback
 * (may now destroy/reuse CEP and QP)	(may now destroy/reuse CEP and QP)
 *
 * (server processes other connections)
 * (server now ready to exit)
 * iba_cm_cancel on listening CEP
 * (CM cleans up pending inbound connections)
 * FCM_CONNECT_CANCEL callback
 * (may now destroy/reuse listening CEP)
 *
 * -----------------------------------------------------------------------------
 * The Basic Sequence for connection rejection by server is as follows:
 * (this assumes ASYNC_ACCEPT and TIMEWAIT_CALLBACK are enabled)
 * iba_cm_create_cep					iba_cm_create_cep
 * iba_cm_listen
 * 										iba_cm_prepare_request
 * 										iba_cm_connect
 * FCM_CONNECT_REQUEST callback
 * iba_cm_reject
 * 										FCM_CONNECT_REJECT callback
 * 										(may now destroy/reuse CEP and QP)
 * -----------------------------------------------------------------------------
 * The Basic Sequence for connection rejection by client is as follows:
 * (this assumes ASYNC_ACCEPT and TIMEWAIT_CALLBACK are enabled)
 * iba_cm_create_cep					iba_cm_create_cep
 * iba_cm_listen
 * 										iba_cm_prepare_request
 * 										iba_cm_connect
 * FCM_CONNECT_REQUEST callback
 * iba_cm_process_request
 * iba_cm_accept w/ reply data
 * (iba_cm_accept returns new CEP)
 * 										FCM_CONNECT_REPLY callback
 *										iba_cm_reject
 * 										(may now destroy/reuse CEP and QP)
 * FCM_CONNECT_REJECT callback
 * (may now destroy/reuse CEP and QP)
 * -----------------------------------------------------------------------------
 * The Sequence for stale connection rejection by server is as follows:
 * (this only occurs if client reuses QPs before the timewait period expires)
 * (this assumes ASYNC_ACCEPT and TIMEWAIT_CALLBACK are enabled)
 * iba_cm_create_cep					iba_cm_create_cep
 * iba_cm_listen
 * 										iba_cm_prepare_request
 * 										iba_cm_connect
 * (internally CM identifies stale QP)
 * 										FCM_CONNECT_REJECT callback
 * 										w/RC_STALE_CONN reason
 * 					(timewait delay occurs)
 * 										FCM_DISCONNECTED callback
 * 										(may now destroy/reuse CEP and QP)
 * -----------------------------------------------------------------------------
 * The Sequence for stale connection rejection by client is as follows:
 * (this only occurs if server reuses QPs before the timewait period expires)
 * (this assumes ASYNC_ACCEPT and TIMEWAIT_CALLBACK are enabled)
 * iba_cm_create_cep					iba_cm_create_cep
 * iba_cm_listen
 * 										iba_cm_prepare_request
 * 										iba_cm_connect
 * FCM_CONNECT_REQUEST callback
 * iba_cm_process_request
 * iba_cm_accept w/ reply data
 * (iba_cm_accept returns new CEP)
 * 										(internally CM identifies stale QP)
 * 										(CM discards REP, waits for a good one)
 * 										(CM will retry until timeout or success)
 * FCM_CONNECT_REJECT callback
 * 	w/RC_STALE_CONN reason
 * 					(timewait delay occurs)
 * FCM_DISCONNECTED callback
 * (may now destroy/reuse CEP and QP)
 *
 * -----------------------------------------------------------------------------
 * The Basic Sequence for connection timeout by client is as follows:
 * (this assumes ASYNC_ACCEPT and TIMEWAIT_CALLBACK are enabled)
 * (typically this occurs if there is no Server running, servers should
 * not simply ignore CONNECT_REQUEST callbacks, as this will cause CEPs to
 * be consumed until the server exits)
 * Server								Client
 * iba_cm_create_cep					iba_cm_create_cep
 * iba_cm_listen
 * 										iba_cm_prepare_request
 * 										iba_cm_connect
 * FCM_CONNECT_REQUEST callback
 * (server fails to respond)
 * 										FCM_CONNECT_TIMEOUT callback
 * 										(may now destroy/reuse CEP and QP)
 *
 * -----------------------------------------------------------------------------
 * The Basic Sequence for connection timeout by server is as follows:
 * (this assumes ASYNC_ACCEPT and TIMEWAIT_CALLBACK are enabled)
 * (typically this occurs if the client has exited during connection, clients
 * should not simply ignore CONNECT_REPLY callbacks, as this will cause CEPs
 * to be consumed until the client exits)
 * Server								Client
 * iba_cm_create_cep					iba_cm_create_cep
 * iba_cm_listen
 * 										iba_cm_prepare_request
 * 										iba_cm_connect
 * FCM_CONNECT_REQUEST callback
 * iba_cm_process_request
 * iba_cm_accept w/ reply data
 * (iba_cm_accept returns new CEP)
 * 										FCM_CONNECT_REPLY callback
 * 										(client fails to respond)
 * FCM_CONNECT_TIMEOUT callback
 * (may now destroy/reuse CEP and QP)
 *
 *
 * *****************************************************************************
 * Peer to Peer RC Connections
 * -----------------------------------------------------------------------------
 * The Basic Sequence for connection establishment is as follows:
 * (ater initial iba_cm_connect_peer call, CM will internally assign one
 * Peer a passive (listener-like) and one peer an active role.
 * The sequence below shows the left most peer in the passive role)
 * (this assumes ASYNC_ACCEPT and TIMEWAIT_CALLBACK are enabled)
 * Peer									Peer
 * iba_cm_create_cep					iba_cm_create_cep
 * iba_cm_connect_peer
 * 										iba_cm_prepare_request
 * 										iba_cm_connect_peer
 * FCM_CONNECT_REQUEST callback
 * iba_cm_process_request
 * iba_cm_accept w/ reply data
 * (iba_cm_accept returns new CEP)
 * 										FCM_CONNECT_REPLY callback
 *										iba_cm_process_reply
 * 										iba_cm_accept w/Rtu data
 * 										(may now send on QP)
 * FCM_CONNECT_ESTABLISHED callback
 * iba_cm_prepare_rts
 * (may now send on QP)
 *
 * (disconnect could be initiated by either side, right Peer initiates below):
 *										iba_cm_disconnect w/request data
 * FCM_DISCONNECT_REQUEST callback
 * iba_cm_disconnect w/reply data
 * 
 * 					(timewait delay occurs)
 * FCM_DISCONNECTED callback			FCM_DISCONNECTED callback
 * (may now destroy/reuse CEP and QP)	(may now destroy/reuse CEP and QP)
 *
 * *****************************************************************************
 * Alternate Path Migration
 * -----------------------------------------------------------------------------
 * APM is allowed on a Client at any time.  The Client drives all aspects of APM
 * On a server, the CM_FLAG_APM must be set via iba_modify_cep.  This can be
 * set in the listener anytime prior to iba_cm_accept, in which case it will be
 * inherited by all inbound ceps for that listener.  It is generally best to set
 * it immediately after iba_cm_create_cep.  If CM_FLAG_APM is not
 * set the CM will automatically reject inbound APM packets and the server
 * will not get any APM callbacks.
 *
 * The Basic Sequence for loading an alternate path is as follows:
 * Server								Client
 *										iba_cm_altpath_request
 * FCM_ALTPATH_REQUEST callback
 * iba_cm_process_altpath_request
 * modify QP with Alt path, Rearm APM
 * iba_cm_altpath_reply
 * 	w/ APStatus = APS_PATH_LOADED
 * 										FCM_ALTPATH_REPLY callback
 *										iba_cm_process_altpath_reply
 * 										modify QP with Alt path, Rearm APM
 * -----------------------------------------------------------------------------
 * Any APStatus other than APS_PATH_LOADED is considered a rejection.
 * The special case of APS_UNSUPPORTED_REQ indicates the server does not
 * support APM at all, in which case the client will get an error return on
 * future attempts at iba_cm_altpath_request and the server's stack will
 * automatically reject any future alternate path requests.
 *
 * The Basic Sequence for the server rejecting an alternate path is as follows:
 * Server								Client
 *										iba_cm_altpath_request
 * FCM_ALTPATH_REQUEST callback
 * iba_cm_altpath_reply
 * 	w/ APStatus != APS_PATH_LOADED
 * 										FCM_ALTPATH_REJECT
 * -----------------------------------------------------------------------------
 * The IBTA 1.1 CM protocol has limitations in associating LAPs and APR
 * messages since the APR has no unique identifier to associate it with
 * the proper LAP.  Hence a late APR could be treated as a response to a later
 * LAP and the endpoints would end up with inconsistent alternate paths.
 * To avoid this issue, if a LAP times out, the client endpoint will
 * consider this an indication the remote end point does not support
 * APM. In which case the client will get an error return on
 * future attempts at iba_cm_altpath_request.
 *
 * The Basic Sequence for the server not responding to an alternate path
 * is as follows:
 * Server								Client
 *										iba_cm_altpath_request
 * FCM_ALTPATH_REQUEST callback
 * server fails to respond
 * 										FCM_ALTPATH_TIMEOUT
 * -----------------------------------------------------------------------------
 * The iba_cm_migrated call must be made by the ULP in reponse to
 * a AsyncEventPathMigrated.  However in the future this requirement may be
 * removed for the server side, at which time the iba_cm_migrated call will
 * become a noop for server CEPs.
 *
 * The Basic Sequence for a path migration is as follows:
 * Server								Client
 * HCA migrate protocol					HCA migrate protocol
 * AsyncEventPathMigrated callback		AsyncEventPathMigrated callback
 * iba_cm_migrated						iba_cm_migrated
 *											or
 *										iba_cm_migrated_reload
 *										iba_cm_altpath_request
 *
 * *****************************************************************************
 * Other Features
 * -----------------------------------------------------------------------------
 * The iba_cm_reject and iba_cm_accept calls are intended for use as the direct
 * result of a callback (as shown above).  In addition if a client/server
 * decides to terminate a connection establishment sequence, the iba_cm_cancel
 * call can be used.  This will trigger the cancellation or rejection of the
 * connection (action taken depends on the state of the CEP).
 * After the cancel operations complete, a FCM_CONNECT_CANCEL callback will
 * be provided at which time the application can safely destroy/reuse the CEP
 * and QP.
 *
 * If the iba_cm_cancel is not permitted for the present state (for example if
 * the connection is already established), the FINVALID_STATE error code is
 * returned and the application should use iba_cm_disconnect with a request.
 * Should iba_cm_disconnect also return FINVALID_STATE, the disconnect is in
 * progress and the application will ultimately get an FCM_DISCONNECTED callback
 * when the disconnect sequence has completed.
 *
 * This approach allows the CEP and QP handling to be done directly as a
 * result of callbacks and helps reduce concurrency issues in the
 * application.  Hence for a typical application, when a higher level
 * decides to cancel a connection if should simply do:
 * 	if (iba_cm_cancel(cep) != FSUCCESS)
 *      (request optional, can be NULL)
 * 		(void)iba_cm_disconnect(cep, &request, NULL);
 * This will ensure an appropriate final callback will occur for the CEP
 * to trigger application cleanup.
 *
 * If the iba_cm_cancel happens to race with a callback thread, functions in the
 * callback such as iba_cm_accept/iba_cm_reject may return FCM_CONNECT_CANCEL.
 * In which case that callback need do not more work on the CEP/QP and a future
 * FCM_CONNECT_CANCEL callback will occur to allow final cleanup.
 * -----------------------------------------------------------------------------
 * As has been implied above, when using the TIMEWAIT_CALLBACK and ASYNC_ACCEPT
 * options (recommended), the following events are considered the final
 * events for a CEP.  When these events occur, the application can destroy or
 * reuse the CEP and corresponding QP.
 * 	FCM_DISCONNECTED/FCM_DISCONNECT_TIMEOUT callback (both same value)
 * 	iba_cm_reject returns FSUCCESS
 * 	FCM_CONNECT_REJECT callback/return from iba_cm_accept (except RC_CONN_STALE)
 * 	FCM_CONNECT_CANCEL callback/return from iba_cm_accept
 * 	FCM_CONNECT_TIMEOUT callback/return from iba_cm_accept
 *	FCM_CA_REMOVED callback
 * -----------------------------------------------------------------------------
 * CEP handles are validated on all calls.  Hence if a CEP has been destroyed in
 * another thread, any subsequenct calls against the CEP will return
 * FCM_INVALID_HANDLE.
 *
 * 
 * *****************************************************************************
 * Typical RC/UC/RD Connection setup algorithms:
 * -----------------------------------------------------------------------------
 * Server:
 * 	iba_cm_create_cep
 * 	uint32 value = 1;
 * 	iba_cm_modify_cep(cep, CM_FLAG_ASYNC_ACCEPT, (char*)&value,
 * 					sizeof(uint32), 0);
 * 	iba_cm_modify_cep(cep, CM_FLAG_TIMEWAIT_CALLBACK, (char*)&value,
 * 					sizeof(uint32),0);
 * 	iba_cm_modify_cep(cep, CM_FLAG_APM, (char*)&value,
 * 					sizeof(uint32),0);	// allow APM
 * 	value = max pending connect requests on CEP queue
 * 	iba_cm_modify_cep(cep, CM_FLAG_LISTEN_BACKLOG, (char*)&value,
 * 					sizeof(uint32), 0);
 * 	if private data is to be part of bound listening address
 * 		uint8 pattern[] = data pattern to listen for in REQ
 * 		iba_cm_modify_cep(cep, CM_FLAG_LISTEN_DISCRIM, pattern,
 *				length of pattern, offset within REQ private data)
 * 	create/get an application structure for use as listen context (if any)
 * 	iba_cm_listen(cep, ListenRequest, callback, context)
 * -----------------------------------------------------------------------------
 * Client or Peer to Peer:
 *	iba_cm_create_cep
 * 	uint32 value = 1;
 * 	iba_cm_modify_cep(cep, CM_FLAG_ASYNC_ACCEPT, (char*)&value,
 * 					sizeof(uint32), 0);
 * 	iba_cm_modify_cep(cep, CM_FLAG_TIMEWAIT_CALLBACK, (char*)&value,
 * 					sizeof(uint32),0);
 * 	obtain a IB_PATH_RECORD for path to server/peer
 * 	iba_cm_prepare_request
 * 	create/get a QP
 * 	compose rest of Request based on QPN and desired SID
 * 	move QP to Init (using QpAttrInit from iba_cm_prepare_request)
 * 	create/get an application structure for use as context (if any)
 * 	iba_cm_connect(cep, Request, callback, context)
 * 		or
 * 	iba_cm_connect_peer(cep, Request, callback, context)
 * *****************************************************************************
 *
 * Typical Connection teardown algorithms:
 * -----------------------------------------------------------------------------
 * Client, Server or Peer to Peer:
 * 	move QP to Error
 * 	compose a  DisconnectRequest
 * 	iba_cm_disconnect(cep, DisconnectRequest, NULL)
 *
 * Some applications may chose to move the QP to QPStateSendQDrain and wait for
 * the send Q to drain prior to performing the connection teardown sequence
 * above. This way any final transfers on the connection will be guaranteed
 * to have completed.
 *
 * Generally its desirable to move the QP through the error state on its way
 * to Reset or being destroyed.  This will cause all posted WQEs to be returned
 * via completion callbacks with a WRStatusWRFlushed completion code.
 * Most applications will actually delay the final QP destroy until the last
 * such completion for the QP occurs.
 *
 * *****************************************************************************
 * Typical RC/UC/RD Callback handling algorithms:
 * These algorithms are focused on CM and QP operations required in each
 * callback.  In addition there may be additional application specific functions
 * (such as maintaining application state for a CEP, etc).
 * -----------------------------------------------------------------------------
 * FCM_CONNECT_REQUEST	- a REQ was received info->Info.Request supplied
 * This will occur on a Server or a Peer to Peer CEP
 *
 * Algorithm:
 * (this assumes ASYNC_ACCEPT and TIMEWAIT_CALLBACK are enabled)
 * 	verify Info.Request is valid/acceptable to the Server
 *	validate alternate path
 *		if APM not supported, pass CM_REP_FO_NOT_SUPPORTED below
 *		if APM supported but path not acceptable, pass CM_REP_FO_REJECTED_ALT
 *		(not if CA does not support APM or CEP does not have CM_FLAG_APM set
 *		iba_cm_process_request will automatically handle)
 *	iba_cm_process_request(.., CM_REP_FO_*, ...)
 *	further adjust Reply and corresponding QpAttr if desired
 *	verify Reply->Arb is appropriate for RDMA Read needs
 * 	create/get a QP
 *	fill in rest of reply (QPN, RnRRetryCount, PrivateData)
 *	for server (eg. non-peer), move QP to Init:
 *		set QpAttrInit->AccessControl
 * 		move QP to Init state using QpAttrInit from iba_cm_process_request
 * 	move QP to RTR state using QpAttrRtr from iba_cm_process_request
 * 	create/get an application structure for use as context (if any)
 * 	post recieve buffers
 * 	if all the above is successful
 * 		// async accept ignores 3rd argument of iba_cm_accept
 * 		iba_cm_accept(cep, Reply, NULL, callback, context, &newcep)
 * 		if failed
 * 			iba_cm_reject(cep, RejectInfo)
 * 			move QP to Error or Reset state (as needed)
 * 			destroy/free QP (as needed)
 * 			free context structure (as needed)
 * 		else
 * 			// at this point the application could begin to receive data on QP
 * 			// such data receipt could race with the FCM_CONNECT_ESTABLISHED
 * 			// callback
 * 			// any future callbacks against newcep will be given to
 * 			// callback with context
 * 	else
 * 		iba_cm_reject(cep, RejectInfo)
 * 		move QP to Error or Reset state (as needed)
 * 		destroy/free QP (as needed)
 * 		free context structure (if any)
 * -----------------------------------------------------------------------------
 * FCM_CONNECT_ESTABLISHED - a RTU was received info->Info.Rtu supplied
 * 						  In some cases an RTU is optional, in which case
 * 						  this callback may occur with a zeroed Rtu
 * 						  applications should not depend on the PrivateData
 * 						  in the RTU.
 * This will occur on a Server or a Peer to Peer CEP
 *
 * Algorithm:
 *  Info.Rtu may contain private data, but cannot be depended on per IBTA
 *	iba_cm_prepare_rts
 * 	move QP to RTS
 * 	if above fails
 * 		move QP to Error
 * 		iba_cm_disconnect(cep, DisconnectRequest, NULL)
 * 	else
 * 		QP is now fully ready for use
 * 		application may now perform PostSend
 * -----------------------------------------------------------------------------
 * FCM_CONNECT_REPLY	- a REP was received info->Info.Reply supplied
 * This will occur on a Client or a Peer to Peer CEP
 *
 * Algorithm:
 * 	verify Info.Reply is valid/acceptable to the Client/Peer
 *	verify Info.Reply.Arb* meets the RDMA Read needs of the Client/Peer
 *	iba_cm_process_reply
 * 	post recieve buffers
 * 	compose a Rtu (can be all zeros)
 * 	move QP to RTR and RTS state using data from iba_cm_process_reply
 * 		(Note: AckTimeout used for QP RTS is computed as:
 * 			REQ's AckTimeout+Target Ack Delay -local CA Ack Delay
 * 				or
 * 			2*PktLifetime + TargetAckDelay)
 * 	if all the above is successful
 * 		// async accept ignores 3rd argument of iba_cm_accept
 * 		// RTU argument is optional, only needed if have RTU private data
 * 		iba_cm_accept(cep, Rtu, NULL, NULL, NULL, NULL)
 * 		if failed
 * 			iba_cm_reject(cep, RejectInfo)
 * 			move QP to Error or Reset
 *	 		Destroy/free QP
 * 			iba_cm_destroy_cep
 * 			free context structure
 * 		else
 * 			// application can now begin to send and receive data
 * 	else
 * 		iba_cm_reject(cep, RejectInfo)
 * 		move QP to Error or Reset
 * 		Destroy/free QP
 * 		iba_cm_destroy_cep
 * 		free context structure (if any)
 * -----------------------------------------------------------------------------
 * FCM_CONNECT_TIMEOUT	- timeout waiting for REP or RTU for connection
 * 						  no additional info->Info supplied
 * This will occur on a Server, client or a Peer to Peer CEP
 * 
 * Algorithm:
 * 	move QP to Error or Reset state
 * 	Destroy/free QP
 * 	free context structure
 * 	iba_cm_destroy_cep
 * -----------------------------------------------------------------------------
 * FCM_CONNECT_REJECT	- a REJ was received info->Info.Reject supplied
 * 						  For the RC_STALE_CONN rejection code,
 * 						  when CM_FLAG_TIMEWAIT_CALLBACK is set for a
 * 						  client/active peer or both CM_FLAG_ASYNC_ACCEPT and
 * 						  CM_FLAG_TIMEWAIT_CALLBACK are set for a
 * 						  server/passive peer, a FCM_DISCONNECTED
 *						  callback will later occur at the end of Timewait
 *						  For other rejection codes (or server/passive peer
 *						  without both flags set), this is the final
 *						  callback for the CEPs connection sequence
 * This will occur on a Server, client or a Peer to Peer CEP
 * 
 * Algorithm (assuming TIMEWAIT and ASYNC ACCEPT enable):
 * 	move QP to Error or Reset state
 *	if info->Info.Reject != RC_STALE_CONN
 * 		Destroy/free QP
 * 		iba_cm_destroy_cep
 * 		free context structure
 * -----------------------------------------------------------------------------
 * FCM_ALTPATH_REQUEST	- client has requested server to load an alternate path
 *						  a LAP was received info->Info.AltPathRequest supplied
 * This will occur on a Server, a passive Peer to Peer CEP
 * 
 * Algorithm:
 * 	validate new alternate path
 *	iba_cm_process_altpath_request
 * 	modify QP, RTS to RTS, load new alternate path, QP migrate state to Rearm
 * 	if any of above fails
 * 		iba_cm_altpath_reply w/ APStatus != APS_PATH_LOADED
 * 	else
 * 		iba_cm_altpath_reply w/ APStatus = APS_PATH_LOADED
 * -----------------------------------------------------------------------------
 * FCM_ALTPATH_REPLY	- server has accepted alternate path requested
 *						  a APR was received info->Info.AltPathReply supplied
 * This will occur on a client, or a active Peer to Peer CEP
 * 
 * Algorithm:
 *	iba_cm_process_altpath_reply
 * 	modify QP, RTS to RTS, load new alternate path, QP migrate state to Rearm
 * 	if fails
 * 		iba_cm_disconnect
 * -----------------------------------------------------------------------------
 * FCM_ALTPATH_REJECT	- server has rejected alternate path requested
 *						  a APR was received info->Info.AltPathReply supplied
 * This will occur on a client, or a active Peer to Peer CEP
 * 
 * Algorithm:
 * 	no action needed
 * 	could try a iba_cm_altpath_request with a different path
 * -----------------------------------------------------------------------------
 * FCM_ALTPATH_TIMEOUT	- server has not responded to alternate path requested
 * This will occur on a client, or a active Peer to Peer CEP
 * 
 * Algorithm:
 * 	if server previously reponded to APM or ULP requires/expects APM
 * 		it is best to iba_cm_disconnect
 * 	alternately could proceed and assume server does not support APM
 * -----------------------------------------------------------------------------
 * AsyncEventPathMigrated  - QP Async Event callback from verbs interface
 * This can occur on a Server, client or a Peer to Peer CEP for which an
 * alternate path has been loaded into the QP
 *
 * Server/Passive Algorithm (or if Client does not want a new alternate path):
 * iba_cm_migrated
 *
 * Client/Active Peer Algorithm:
 * iba_cm_migrated_reload
 * review/determine proper new alternate path (if any)
 * iba_cm_altpath_request - restore a new alternate path
 * -----------------------------------------------------------------------------
 * FCM_DISCONNECT_REQUEST - a DREQ was received info->Info.DisconnectRequest
 * 						  supplied.  Beware there are situations where a DREQ
 * 						  is internally generated by the CM, in which case the
 * 						  private data will be all 0s
 * This will occur on a Server, client or a Peer to Peer CEP
 * 
 * Algorithm:
 * 	move QP to Error
 * 	optionally process PrivateData
 * 	iba_cm_disconnect(cep, NULL, DisconnectReply)
 * -----------------------------------------------------------------------------
 * FCM_DISCONNECT_REPLY	- a DREP was received info->Info.DisconnectReply
 * 						  supplied
 * This will occur on a Server, client or a Peer to Peer CEP
 * which has initiated disconnect by
 * iba_cm_disconnect(cep, DisconnectRequest, NULL)
 * 
 * Algorithm:
 * 	// QP should already be in Error state
 * 	optionally process PrivateData
 * -----------------------------------------------------------------------------
 * FCM_DISCONNECTED		- disconnect sequence has completed (possibly with
 * 						  a timeout waiting for DREP), no additional
 * 						  info->Info supplied.
 * 						  (same value as FCM_DISCONNECT_TIMEOUT)
 * This will occur on a Server, client or a Peer to Peer CEP
 * 
 * Algorithm:
 * 	move QP to Reset
 *	destroy/free QP
 * 	iba_cm_destroy_cep
 * 	free context structure
 * -----------------------------------------------------------------------------
 * FCM_CA_REMOVED		- CA was removed.  Connection torn down.
 * 						  no additional info->Info supplied.
 * This will occur on a Server, client or a Peer to Peer CEP
 *
 * Algorithm:
 *  CA is gone, no QP actions possible
 *  cleanup QP resources, buffers
 *  iba_cm_destroy_cep
 *  free context structure
 * -----------------------------------------------------------------------------
 * FCM_CONNECT_CANCEL	- iba_cm_cancel has completed cancelation of
 *						  connect/listen no additional info->Info supplied
 * For a listener CEP:
 * 	iba_cm_destroy_cep
 * 	free listener context structure
 *
 * For a client, server or Peer to Peer CEP:
 * 	move QP to Error or Reset state
 * 	Destroy/free QP
 * 	iba_cm_destroy_cep
 * 	free context structure
 *
 * *****************************************************************************
 * Client Server UD Connections
 * -----------------------------------------------------------------------------
 * The Basic Sequence for connection establishment is as follows:
 * Server								Client
 * iba_cm_create_cep					iba_cm_create_cep
 * iba_cm_sidr_register
 * 										iba_cm_sidr_query
 * optional FSIDR_REQUEST callback
 * iba_cm_sidr_response w/ response data
 * 										FSIDR_RESPONSE callback
 * 										may destroy or re-use CEP
 *
 * For a UD QP, IBTA does not require any specific state transitions.
 * Depending on the application, the UD QP could be created once at the
 * start of the applications and moved immediately to RTS, or a separate
 * UD QP could be created per inbound SIDR_REQUEST.  Since there is no
 * disconnect protocol defined for UD QPs, the application must determine
 * how and when it will destroy the QP
 *
 * iba_cm_sidr_deregister will remove the effects of iba_cm_sidr_register.
 * After it the server CEP may be destroyed.
 *
 * To reject an inbound FSIDR_REQUEST, use iba_cm_sidr_response and set
 * SidrResponse.Status to an appropriate SIDR_RESP_STATUS value.
 *
 *
 * *****************************************************************************
 * Typical Callback handling algorithms:
 * FSIDR_REQUEST		- a SIDR_REQ was received info->Info.SidrRequest
 * 						  supplied
 * Only occurs on CEP which was iba_cm_sidr_register()'ed with a callback
 * specified.
 * Otherwise the CM will internally reply to SIDR_REQ messages using the
 * data from iba_cm_sidr_register without involving the application
 *
 * Algorithm:
 * 	compose response to SIDR_REQ
 * 	iba_cm_sidr_response(cep, SidrResponse)
 * -----------------------------------------------------------------------------
 * FSIDR_RESPONSE		- a SIDR_RESP was received info->Info.SidrResponse
 * 						  supplied
 * Only occurs on CEP which did iba_cm_sidr_query.
 *
 * Algorithm:
 * 	application specific handling of SidrResponse
 * 	CEP may now be freed or re-used for another
 *		iba_cm_sidr_query/iba_cm_connect/ib_cm_listen, etc
 * -----------------------------------------------------------------------------
 * FSIDR_RESPONSE_ERR	- a SIDR_RESP was received info->Info.SidrResponse
 * 						  supplied, info->Info.SidrResponse.Status != 0
 * Only occurs on CEP which did iba_cm_sidr_query.
 *
 * Algorithm:
 * 	application specific handling of SidrResponse
 * 	CEP may now be freed or re-used for another
 *		iba_cm_sidr_query/iba_cm_connect/ib_cm_listen, etc
 * -----------------------------------------------------------------------------
 * FSIDR_REQUEST_TIMEOUT- timeout waiting for SIDR_RESP no additional
 * 						  info->Info supplied
 * Only occurs on CEP which did iba_cm_sidr_query.
 *
 * Algorithm:
 * 	application specific handling of SidrResponse
 * 	CEP may now be freed or re-used for another
 *		iba_cm_sidr_query/iba_cm_connect/ib_cm_listen, etc
 */


#include "iba/public/datatypes.h"		/* Portable datatypes */
#include "iba/vpi.h"
#include "iba/stl_types.h"		/* IB-defined types */
#include "iba/ib_sa_records_priv.h"	/* PATH_RECORD */
#include "iba/ib_status.h"		/* IB-defined status code */

#if defined (__cplusplus)
extern "C" {
#endif

/*****************************************************************************
 * CM-specific error & status code
 */

#define FCM_BASE_STATUS				0 
#define FCM_NOT_INITIALIZED			(SET_FSTATUS(FERROR, IB_MOD_CM, 1))
#define FCM_INVALID_HANDLE			(SET_FSTATUS(FERROR, IB_MOD_CM, 2))
#define FCM_ADDR_INUSE				(SET_FSTATUS(FERROR, IB_MOD_CM, 3))
#define FCM_INVALID_EVENT			(SET_FSTATUS(FERROR, IB_MOD_CM, 4))
#define FCM_ALREADY_DISCONNECTING	(SET_FSTATUS(FERROR, IB_MOD_CM, 5))
#define FCM_CONNECT_DESTROY			(SET_FSTATUS(FERROR, IB_MOD_CM, 6))

#define FCM_CONNECT_TIMEOUT			(SET_FSTATUS(FTIMEOUT, IB_MOD_CM, 10))
#define FSIDR_REQUEST_TIMEOUT		(SET_FSTATUS(FTIMEOUT, IB_MOD_CM, 11))

#define FCM_CONNECT_REJECT			(SET_FSTATUS(FREJECT, IB_MOD_CM, 13))
#define FCM_CONNECT_CANCEL			(SET_FSTATUS(FCANCELED, IB_MOD_CM, 14))

#define FCM_CONNECT_REPLY			(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 20))
#define FCM_DISCONNECT_REQUEST		(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 21))
#define FCM_DISCONNECT_REPLY		(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 22))
#define FCM_CONNECT_REQUEST			(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 23))
#define FCM_CONNECT_ESTABLISHED		(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 24))
#define FCM_DISCONNECTED			(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 25))
#define FSIDR_REQUEST				(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 26))
#define FSIDR_RESPONSE				(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 27))
#define FSIDR_RESPONSE_ERR			(SET_FSTATUS(FERROR, IB_MOD_CM, 28))
#define FCM_CA_REMOVED				(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 29))
#define FCM_ALTPATH_REQUEST			(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 30))
#define FCM_ALTPATH_REPLY			(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 31))
#define FCM_ALTPATH_REJECT			(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 32))
#define FCM_ALTPATH_TIMEOUT			(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 33))

/* Disconnect timeout is the same as disconnected since the user does not
 * need to do any QP actions specific to the DREP timeout
 * instead any QP actions should occur after the timewait completes
 */
#define FCM_DISCONNECT_TIMEOUT		FCM_DISCONNECTED

#define FCM_WAIT_OBJECT0			(SET_FSTATUS(FSUCCESS, IB_MOD_CM, 64))


/******************************************************************************
 * User is allowed to piggyback private data along with CM messages.
 * The following defined the max length in bytes of user data
 * allowed for different types of CM messages
 */

/*
 * Private/user data length in bytes
 */
#define CM_REQUEST_INFO_USER_LEN		92
#define CM_REPLY_INFO_USER_LEN			196
#define CM_REJECT_INFO_USER_LEN			148
#define CM_RTU_INFO_USER_LEN			224
#define CM_DREQUEST_INFO_USER_LEN		220
#define CM_DREPLY_INFO_USER_LEN			224
#define SIDR_REQ_INFO_USER_LEN			216
#define SIDR_RESP_INFO_USER_LEN			136
#define CM_LAP_INFO_USER_LEN			168
#define CM_APR_INFO_USER_LEN			148

#define CM_GEN_INFO_USER_LEN			224	/* Generic */
#define CM_REJ_ADD_INFO_LEN				72 	/* Additional reject info length */
#define CM_APR_ADD_INFO_LEN				72  /* Additional apr info length */

/*****************************************************************************
 * Forward decl.
 */
struct _CM_CONN_INFO;

typedef struct _EVENT* EVENT_HANDLE;


/*****************************************************************************
 * CM callback prototype
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object. 
 *
 *	info			- information about the event
 *					  pointer is valid only duration of this call
 *
 *	Context			- user's context for CEP (as was specified in
 *							iba_cm_connect, iba_cm_accept, iba_cm_listen, etc)
 *
 * The info->Status indicates the callback event as follows:
 * FCM_CONNECT_REQUEST	- a REQ was received info->Info.Request supplied
 * FCM_CONNECT_REPLY	- a REP was received info->Info.Reply supplied
 * FCM_CONNECT_ESTABLISHED - a RTU was received info->Info.Rtu supplied
 * 						  In some cases an RTU is optional, in which case
 * 						  this callback may occur with a zeroed Rtu
 * 						  applications should not depend on the PrivateData
 * 						  in the RTU.
 * FCM_CONNECT_REJECT	- a REJ was received info->Info.Reject supplied
 * 						  For the RC_STALE_CONN rejection code,
 * 						  when CM_FLAG_TIMEWAIT_CALLBACK is set for a
 * 						  client/active peer or both CM_FLAG_ASYNC_ACCEPT and
 * 						  CM_FLAG_TIMEWAIT_CALLBACK are set for a
 * 						  server/passive peer, a FCM_DISCONNECTED
 *						  callback will later occur at the end of Timewait
 *						  For other rejection codes (or server/passive peer
 *						  without both flags set), this is the final
 *						  callback for the CEPs connection sequence
 * FCM_DISCONNECT_REQUEST - a DREQ was received info->Info.DisconnectRequest
 * 						  supplied.  Beware there are situations where a DREQ
 * 						  is internally generated by the CM, in which case the
 * 						  private data will be all 0s
 * FCM_DISCONNECT_REPLY	- a DREP was received info->Info.DisconnectReply
 * 						  supplied
 * FCM_CONNECT_CANCEL	- iba_cm_cancel has completed cancelation of
 *						  connect/listen no additional info->Info supplied
 * FCM_DISCONNECTED		- disconnect sequence has completed (possibly with
 * 						  a timeout waiting for DREP), no additional
 * 						  info->Info supplied.
 * 						  (same value as FCM_DISCONNECT_TIMEOUT)
 * FCM_CONNECT_TIMEOUT	- timeout waiting for REP or RTU for connection
 * 						  no additional info->Info supplied
 * FSIDR_REQUEST		- a SIDR_REQ was received info->Info.SidrRequest
 * 						  supplied
 * FSIDR_RESPONSE		- a SIDR_RESP was received info->Info.SidrResponse
 * 						  supplied
 * FSIDR_RESPONSE_ERR	- a SIDR_RESP was received info->Info.SidrResponse
 * 						  supplied, info->Info.SidrResponse.Status != 0
 * FSIDR_REQUEST_TIMEOUT- timeout waiting for SIDR_RESP no additional
 * 						  info->Info supplied
 * FCM_CA_REMOVED		- CA was removed.  Connection torn down.
 * 						  no additional info->Info supplied.
 *
 * Internally the CM handles all retries, invalid/out of sequence messages
 * and the appropriate state transitions.  callbacks will only be issued
 * once for a given message type and only when valid for the present
 * state.
 */
typedef void (*PFN_CM_CALLBACK)(
				IN IB_HANDLE hCEP,
				IN struct _CM_CONN_INFO* info,
				IN void* Context
				);


/*****************************************************************************
 * CM_CEP_TYPE - 
 * Transport service type enum (used in CM API and on Wire in REQ)
 */
typedef enum {
	CM_RC_TYPE=0,				/* Reliable Connection */
	CM_UC_TYPE=1,				/* Unreliable Connection */
	CM_RD_TYPE=2,				/* Reliable Datagram */
	CM_UD_TYPE=3				/* Unreliable Datagram. 			 */
}CM_CEP_TYPE;


/*****************************************************************************
 * CM_FLAG_XXX - options which can be set for a CEP via iba_cm_modify_cep
 * Some of these are bit flags, but the primary use of this
 * value is as an argument to iba_cm_modify_cep, in which case they need not
 * be truely bit flags.
 */
	/*	Private data is not part of bound address (the default) */
#define CM_FLAG_LISTEN_NODISCRIM		0x01
	/*	Private data is part of bound address */
#define CM_FLAG_LISTEN_DISCRIM			0x02

	/* maximum pending CEPs on a listener or SIDR Registered CEP
	 * value is a uint32 count
	 * If set to 0, the CmMaxBackLog value will be used for the CEP
	 * (and any adjustments to CmMaxBackLog will take effect on all such CEPs)
	 */
#define CM_FLAG_LISTEN_BACKLOG			0x04

	/* Is user providing a callback for inbound SIDR REQ handling
	 * value is a uint32 flag with value of 0 or 1 (1-> enable)
	 */
#define CM_FLAG_SIDR_REGISTER_NOTIFY	0x08

	/* Does user want Passive iba_cm_accept to be Asynchronous?
	 * In which case a separate FCM_CONNECT_ESTABLISHED callback is
	 * given when RTU is received.
	 * It is recommended this option be enabled
	 * Value of this option is inheritted from a listener to the new CEPs
	 * at the time a REQ is accepted.
	 * value is a uint32 flag with value of 0 or 1 (1-> enable)
	 */
#define CM_FLAG_ASYNC_ACCEPT			0x10

	/* User can set their Application Turnaround Time (in usec)
	 * if set to 0, a heuristic average across all applications
	 * will be used.
	 * This value includes both software queuing time and appl processing time.
	 * For a client it is used to influence the timeouts specified
	 * in the connect request.
	 * For a listener this controls when an MRA is issued.
	 * Value of this option is inheritted from a listener to the new CEPs
	 * at the time a REQ is accepted.
	 * value is a uint32 in microseconds
	 */
#define CM_FLAG_TURNAROUND_TIME			0x20

	/* Does user want a FCM_DISCONNECTED callback only after the timewait
	 * period has expired?
	 * It is recommended this option be enabled
	 * Value of this option is inheritted from a listener to the new CEPs
	 * at the time a REQ is accepted.
	 * value is a uint32 flag with value of 0 or 1 (1-> enable)
	 * If set a QP/CEP can be safely reused after any of the following:
	 * 	FCM_DISCONNECTED/FCM_DISCONNECT_TIMEOUT callback (both same value)
	 * 	iba_cm_reject success
	 * 	FCM_CONNECT_REJECT callback/return from iba_cm_accept
	 *			(except RC_STALE_CONN)
	 * 	- exception, for a server/passive peer without CM_FLAG_ASYNC_ACCEPT
	 * 		there will be no FCM_DISCONNECTED callback, timewait proceeds
	 * 		without notification as to when it completed
	 * 	FCM_CONNECT_CANCEL callback/return from iba_cm_accept
	 * 	FCM_CONNECT_TIMEOUT callback/return from iba_cm_accept
	 *  FCM_CA_REMOVED callback
	 * If not set, the user is given no indication as to when the timewait
	 * has completed. The last event for a CEP could be any of the above or:
	 *  FCM_DISCONNECT_REPLY callback
	 *  FCM_DISCONNECT_REQUEST (caller should then iba_cm_disconnect w/ a reply)
	 */
#define CM_FLAG_TIMEWAIT_CALLBACK		0x40

	/* Does server support APM (Alternate Path Migration).
	 * If set the server could get FCM_ALTPATH_REQUEST callbacks
	 * flag is not allowed for clients, client's use of iba_cm_altpath_request
	 * and/or setting of an alternate path in iba_cm_connect will indicate
	 * if client supports APM
	 * Value of this option is inheritted from a listener to the new CEPs
	 * at the time a REQ is accepted.
	 * if server rejects an AltPathRequest with a APStatus of
	 * APS_UNSUPPORTED_REQ, this option is disabled.
	 */
#define CM_FLAG_APM						0x80



/*****************************************************************************
 * CM_REJECTION_CODE - 
 * Rejection reason code
 */
typedef enum {
	RC_NO_QP=1,
	RC_NO_EEC,
	RC_NO_RESOURCES,
	RC_TIMEOUT,
	RC_UNSUPPORTED_REQ,
	RC_INVALID_COMMID,
	RC_INVALID_COMMINST,
	RC_INVALID_SID,
	RC_INVALID_TSTYPE,
	RC_STALE_CONN,
	RC_INVALID_RDC,
	RC_PRIMARY_DGID_REJ,
	RC_PRIMARY_DLID_REJ,
	RC_INVALID_PRIMARY_SL,
	RC_INVALID_PRIMARY_TC,
	RC_INVALID_PRIMARY_HL,
	RC_INVALID_PRIMARY_PR,
	RC_ALTERNATE_DGID,
	RC_ALTERNATE_DLID,
	RC_INVALID_ALTERNATE_SL,
	RC_INVALID_ALTERNATE_TC,
	RC_INVALID_ALTERNATE_HL,
	RC_INVALID_ALTERNATE_PR,
	RC_CMPORT_REDIR,
	RC_INVALID_PATHMTU,
	RC_INSUFFICIENT_RESP_RES,
	RC_USER_REJ,
	RC_RNRCOUNT_REJ
} CM_REJECTION_CODE;

/* convert CM_REJECTION_CODE to a string */
IBA_API const char* iba_cm_rejection_code_msg(uint16 code);


/******************************************************************************
 * CM_APR_STATUS -
 * APR APStatus
 */
typedef enum {
	APS_PATH_LOADED = 0,
	APS_INVALID_COMMID,
	APS_UNSUPPORTED_REQ,
	APS_REJECTED,
	APS_CMPORT_REDIR,
	APS_DUPLICATE_PATH,
	APS_ENDPOINT_MISMATCH,
	APS_REJECT_DLID,
	APS_REJECT_DGID,
	APS_REJECT_FL,
	APS_REJECT_TC,
	APS_REJECT_HL,
	APS_REJECT_PR,
	APS_REJECT_SL
} CM_APR_STATUS;

/* convert CM_APR_STATUS to a string */
IBA_API const char* iba_cm_apr_status_msg(uint16 code);

/******************************************************************************
 * SIDR_RESP_STATUS - 
 * SIDR response status
 */
typedef enum {
	SRS_VALID_QPN=0,
	SRS_SID_NOT_SUPPORTED,
	SRS_S_PROVIDER_REJECTED,
	SRS_QP_UNAVAILABLE,
	SRS_REDIRECT,
	SRS_VERSION_NOT_SUPPORTED
	/* 6-255 reserved */
} SIDR_RESP_STATUS;

/* convert SIDR_RESP_STATUS to a string */
IBA_API const char* iba_cm_sidr_resp_status_msg(uint8 code);


/******************************************************************************
 * CM_CEP_PATHINFO - 
 * Path information use to establish the connection.
 *
 * Path is always presented from the perspective of the Client
 * Hence SLID is Client LID and DLID is Server LID, etc.
 */
typedef struct _CM_CEP_PATHINFO {
	/* If local, the global routing info will not be used below. This include */
	/* the GIDs, HopLimit, TrafficClass and FlowLabel. */
	boolean			bSubnetLocal;
	IB_PATH_RECORD	Path;

} CM_CEP_PATHINFO;


/******************************************************************************
 * CM_CEP_INFO - 
 * Local connection endpoint information.
 * This local information is passed to the remote endpoint
 * during connection establishment.
 */
typedef struct _CM_CEP_INFO {

	/* Client Side Channel Adapter GUID (also known as node GUID) */
	uint64		CaGUID;						/* From NodeInfo.NodeGUID */
	boolean		EndToEndFlowControl;		/* Ca implements end-to-end flow control */
	/* Local Port information */
	uint64		PortGUID;					/* unused on Client */
											/*		(SGID/CaGUID implies) */
											/* returned on Server as Port for */
											/* 		which connection is being */
											/* 		made (based on DGID in REQ) */

	/* Client Side QP & RD information */
	uint32		QKey;						/* QP's QKey (for RD only) */
	uint32		QPN:24;						/* QP Number */
	uint32		AckTimeout:5;				/* Local ACK timeout (Application-defined) */
	uint32		RetryCount:3;				/* Sequence error retry count (Application-defined) */
	uint32		StartingPSN:24;				/* Starting PSN to use on the QP (Application-defined) */
	uint32		AlternateAckTimeout:5;		/* Specify this if the alternate path is used */
	uint32		RnrRetryCount:3;			/* Receiver Not Ready retry count (Application-defined) */

	uint32		LocalEECN:24;
	uint32		OfferedResponderResources:8;	/* Application-defined */
												/* senders's desired */
												/* qpAttr->ResponderResources */

	uint32		RemoteEECN:24;
	uint32		OfferedInitiatorDepth:8;		/* Application-defined */
												/* senders's desired */
												/* qpAttr->InitiatorDepth */

} CM_CEP_INFO;

/******************************************************************************
 * CM_CEP_ADDR - 
 * Connection endpoint address. The CEP object is bound
 * to this address during iba_cm_listen()
 */
typedef struct _CM_CEP_ADDR {
	/* For ListenAddr, SID is used */
	/* For RemoteAddr, ignored */
	/* QPN, EECN fields are ignored, depricated */
	struct {
		uint32		QPN:24;			/* ignored, depricated */
		uint32		Reserved1:8;	/* Padding */
		uint32		EECN:24;		/* ignored, depricated */
		uint32		Reserved2:8;
		uint64		SID;			/* only used in ListenAddr */
	} EndPt;

	/* optionally limit listen to a specific GID or LID */
	/* if 0, will listen on all GIDs/LIDs */
	struct {
		IB_GID		GID;
		IB_LID		LID;
	} Port;
} CM_CEP_ADDR;


/******************************************************************************
 * CM_LISTEN_INFO - 
 * Listen request info. Server specified the parameters to establish 
 * a local listening endpoint to accept incoming connections. 
 * No mapping
 */
typedef struct _CM_LISTEN_INFO {
	uint64			CaGUID;		/* must supply if listening on a specific port */
	CM_CEP_ADDR		ListenAddr;
	CM_CEP_ADDR		RemoteAddr;
} CM_LISTEN_INFO;


/******************************************************************************
 * CM_REQUEST_INFO - 
 * Connect request info. Client specified the parameters to establish 
 * a connection between local and remote endpoint. 
 * Does NOT mapped directly to CMM_REQ
 */
typedef struct _CM_REQUEST_INFO {

	uint64					SID;
	CM_CEP_INFO				CEPInfo;			/* Additonal local endpoint information */
	CM_CEP_PATHINFO			PathInfo;			/* Path info */
	CM_CEP_PATHINFO			AlternatePathInfo;	/* Alternate path info */

	/* Additional user data that is sent across to the remote endpoint for */
	/* interpretion by the remote end user i.e. not interpreted by CM */
	uint8					PrivateData[CM_REQUEST_INFO_USER_LEN];	/* 92 bytes of private data */

} CM_REQUEST_INFO;


/******************************************************************************
 * CM_REP_FAILOVER
 * values for FailoverAccepted field in REPLY_INFO
 */
typedef enum {
	CM_REP_FO_ACCEPTED=0,
	CM_REP_FO_NOT_SUPPORTED,
	CM_REP_FO_REJECTED_ALT
} CM_REP_FAILOVER;

/* convert CM_REP_FAILOVER to a string */
IBA_API const char* iba_cm_rep_failover_msg(uint8 code);

/******************************************************************************
 * CM_REPLY_INFO -
 * Reply info. Use by server to reply to a connect request.
 * Mapped directly to CMM_REP
 */
typedef struct _CM_REPLY_INFO {

	uint32			QKey;			/* QP's QKey (for RD only) */

	uint32			QPN:24;
	uint32			Reserved1:8;

	uint32			EECN:24;
	uint32			Reserved2:8;

	uint32			StartingPSN:24;
	uint32			Reserved3:8;

	uint32			ArbResponderResources:8; /* senders's arbitrated
											 * qpAttr->ResponderResources
											 * receiver should validate and use
											 * to set its qpAttr->InitiatorDepth
											 */
	uint32			ArbInitiatorDepth:8;	/* sender's arbitrated
											 * qpAttr->InitiatorDepth
											 * receiver should validate and use
											 * to set its
											 * qpAttr->ResponderResources
											 */
	uint32			TargetAckDelay:5;
	uint32			FailoverAccepted:2;		/* enum CM_REP_FAILOVER */
	uint32			EndToEndFlowControl:1;
	uint32			RnRRetryCount:3;
	uint32			Reserved4:5;
	/* Server Side Channel Adapter GUID (also known as node GUID) */
	uint64			CaGUID;	/* ignored in server accept, returned to client */
	uint8			PrivateData[CM_REPLY_INFO_USER_LEN];

} CM_REPLY_INFO;

/******************************************************************************
 * CM_REJECT_INFO -
 * Reject info. Use by server to reject a connect request or use
 * by client to reject a connect reply. Mapped directly to CMM_REJ
 */
typedef struct _CM_REJECT_INFO {

	uint32			Reason:16; /* CM_REJECTION_CODE */
	uint32			RejectInfoLen:7;
	uint32			Reserved1:9;
	uint8			RejectInfo[CM_REJ_ADD_INFO_LEN]; /* 72 bytes */
	uint8			PrivateData[CM_REJECT_INFO_USER_LEN]; /* 148 bytes */
} CM_REJECT_INFO;


/******************************************************************************
 * CM_RTU_INFO -
 * Ready-to-use info. Use by client to accept the connect reply. 
 * Mapped directly to CMM_RTU
 */
typedef struct _CM_RTU_INFO {
	uint8			PrivateData[CM_RTU_INFO_USER_LEN]; /* 224 bytes */
} CM_RTU_INFO;


/******************************************************************************
 * CM_ALTPATH_INFO -
 * Load Alternate Path info. Use by client to define a new alternate
 * path for a QP which is using APM for resiliency
 */
typedef struct _CM_ALTPATH_INFO {
	uint8			AlternateAckTimeout;	
	CM_CEP_PATHINFO	AlternatePathInfo;
	uint8			PrivateData[CM_LAP_INFO_USER_LEN]; /* 168 bytes */
} CM_ALTPATH_INFO;

/******************************************************************************
 * CM_ALTPATH_REPLY_INFO -
 * Alternate Path Reply info. Use by server to accept/reject a
 * a new alternate path for a QP which is using APM for resiliency.
 * Also provided in callbacks to client.
 * For client FCM_ALTPATH_REPLY callback with APStatus = APS_PATH_LOADED:
 * 	Alternate.AckTimeout and Alternate.PathInfo will match
 * 	value from original LAP, it is provided in the callback as a convenience so
 * 	application need not keep a local copy
 * For client FCM_ALTPATH_REJECT callback:
 *	APStatus != APS_PATH_LOADED and AddInfo is provided.
 * For server calls to iba_cm_altpath_reply:
 *	only AddInfo field is used
 */
typedef struct _CM_ALTPATH_REPLY_INFO {
	uint16			APStatus;	/* CM_APR_STATUS */
	union {
		struct {	/* only for FCM_ALTPATH_REPLY w/ APStatus == APS_PATH_LOADED */
			CM_CEP_PATHINFO	PathInfo;
			uint8			AckTimeout;
		} Alternate;
		struct {	/* for FCM_ALTPATH_REJECT or iba_altpath_reply */
			uint8			Len;
			uint8			Reserved;
			uint8			Info[CM_APR_ADD_INFO_LEN]; /* 72 bytes */
		} AddInfo;
	} u;
	uint8			PrivateData[CM_APR_INFO_USER_LEN]; /* 148 bytes */
} CM_ALTPATH_REPLY_INFO;


/******************************************************************************
 * CM_DREQUEST_INFO -
 * Disconnect request info. Use by client or server to disconnect an
 * endpoint. Mapped directly to CMM_DREQ
 */
typedef struct _CM_DREQUEST_INFO {
	uint8			PrivateData[CM_DREQUEST_INFO_USER_LEN]; /* 220 bytes */
} CM_DREQUEST_INFO;


/******************************************************************************
 * CM_DREPLY_INFO -
 * Disconnect reply info. Use by client or server to reply to a
 * disconnect request. Mapped directly to CMM_DREP
 */
typedef struct _CM_DREPLY_INFO {
	uint8			PrivateData[CM_DREPLY_INFO_USER_LEN]; /* 224 bytes */
} CM_DREPLY_INFO;


/******************************************************************************
 * SIDR_RESP_INFO -
 * SIDR response info. Use by UD server to response to a
 * SIDR request. Mapped directly to CMM_SIDR_RESP
 */
typedef struct _SIDR_RESP_INFO {

	uint32		QPN:24;
	uint32		Status:8;	/* SIDR_RESP_STATUS */

	uint32		QKey;		
				
	/* 72 bytes of port info for redir or unsupported version. */
	/* This is hidden from the user, in future may need to expose. */

	uint8		PrivateData[SIDR_RESP_INFO_USER_LEN]; /* 136 - 140 bytes */

} SIDR_RESP_INFO;


/******************************************************************************
 * SIDR_REQ_INFO -
 * SIDR request info. Use by UD client to query a UD server. 
 * DOES NOT mapped directly to CMM_SIDR_RESP
 */
typedef struct _SIDR_REQ_INFO {

	uint64					SID;
	uint64					PortGuid;
	uint16					PartitionKey;		/* PKey requested for Service */

	CM_CEP_PATHINFO			PathInfo;			/* Path info */

	uint8					PrivateData[SIDR_REQ_INFO_USER_LEN]; /* 216 bytes */

} SIDR_REQ_INFO;


/******************************************************************************
 * SIDR_REGISTER_INFO -
 * SIDR register info. Use by UD server to register its SID. 
 * No mapping
 */
typedef struct _SIDR_REGISTER_INFO {

	uint64		ServiceID;

	uint32		QPN:24;
	uint32		Reserved1:8;

	uint32		QKey;

} SIDR_REGISTER_INFO;


/******************************************************************************
 * CM_CONN_INFO -
 * Union of the above connection info
 * when provided in a callback, the Status field indicates which structure
 * is valid.
 * When provided in the iba_cm_accept call, the state of the CEP implies the
 * structure to use (eg. Reply on server side, Rtu on client side)
 */
typedef union _CONN_INFO {
	CM_REQUEST_INFO			Request;		/* FCM_CONNECT_REQUEST */
	CM_REPLY_INFO			Reply;			/* FCM_CONNECT_REPLY */
	CM_RTU_INFO				Rtu;			/* FCM_CONNECT_ESTABLISHED */
	CM_REJECT_INFO			Reject;			/* FCM_CONNECT_REJECT */
	CM_ALTPATH_INFO			AltPathRequest;	/* FCM_ALTPATH_REQUEST */
	CM_ALTPATH_REPLY_INFO	AltPathReply;	/* FCM_ALTPATH_REPLY, FCM_ALTPATH_REJECT */
	CM_DREQUEST_INFO		DisconnectRequest;	/* FCM_DISCONNECT_REQUEST */
	CM_DREPLY_INFO			DisconnectReply;	/* FCM_DISCONNECT_REPLY */
	SIDR_REQ_INFO			SidrRequest;	/* FSIDR_REQUEST */
	SIDR_RESP_INFO			SidrResponse;	/* FSIDR_RESPONSE, FSIDR_RESPONSE_ERR */
	CM_LISTEN_INFO			ListenRequest;	/* internal use only */
	SIDR_REGISTER_INFO		SidrRegister;	/* internal use only */
	/* no data for:
	 * 		FCM_CONNECT_TIMEOUT
	 * 		FCM_CONNECT_CANCEL
	 * 		FCM_ALTPATH_TIMEOUT
	 * 		FCM_DISCONNECTED
	 * 		FCM_DISCONNECT_TIMEOUT
	 * 		FSIDR_REQUEST_TIMEOUT
	 * 		FCM_CA_REMOVED
	 */
} CONN_INFO;

typedef struct _CM_CONN_INFO {
	FSTATUS				Status;	/* in output/callback tell field in union */
	CONN_INFO			Info;
} CM_CONN_INFO;


/******************************************************************************
 * Function prototypes
 */

/*
 * iba_cm_create_cep/CmCreateCEP
 *
 *	This routine creates a connection endpoint object (CEP). This object
 *	is used to establish a connection to the remote endpoint or listen for
 *	incoming connections on this endpoint. This object is also used to register
 *	the endpoint to resolve SIDR request
 *
 *
 *
 * INPUTS:
 *
 *	TransportServiceType-	Specify the transport service type, RC, UC, RD, UD
 *	
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	A handle to the CEP object if successful. Otherwise, NULL
 *
 */
typedef IB_HANDLE
(CM_CREATE_CEP)(
	IN	CM_CEP_TYPE	TransportServiceType
	);
IBA_API CM_CREATE_CEP iba_cm_create_cep;
IBA_API CM_CREATE_CEP CmCreateCEP;	/* for backward compatibility */


/*
 * iba_cm_destroy_cep/CmDestroyCEP
 *
 *	This routine destroys the CEP object. This routine is called when the handle
 *	to the CEP object is no longer in use.
 *
 *
 *
 * INPUTS:
 *
 *	hCEP		- Handle to the CEP object that was returned in iba_cm_create_cep().
 *	
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FINVALID_STATE
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *
 */

typedef FSTATUS
(CM_DESTROY_CEP)(
	IN	IB_HANDLE	hCEP
	);
IBA_API CM_DESTROY_CEP iba_cm_destroy_cep;
IBA_API CM_DESTROY_CEP CmDestroyCEP;	/* for backward compatibility */


/*
 * iba_cm_modify_cep/CmModifyCEP
 *
 *	This routine modifies the attribute setting of a connection endpoint object (CEP).
 *
 *
 *
 * INPUTS:
 *
 *	AttrType	- Specify the attribute to modify
 *	AttrValue	- Specify the new value of the attribute	
 *	AttrLen		- Specify the length in bytes of the new attribute value
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	A handle to the CEP object if successful. Otherwise, NULL
 *
 */
typedef FSTATUS
(CM_MODIFY_CEP)(
	IN	IB_HANDLE	hCEP,
	uint32			AttrType,
	const char*		AttrValue,
	uint32			AttrLen,
	uint32			Offset
	);
IBA_API CM_MODIFY_CEP iba_cm_modify_cep;
IBA_API CM_MODIFY_CEP CmModifyCEP;	/* for backward compatibility */

/*
 * iba_cm_prepare_request
 *
 *	This routine is used by a client to help build a Connect Request for
 *	use in iba_cm_connect.  It also builds the QP attributes to move the
 *	QP to Init.
 *
 *
 * INPUTS:
 *
 *	TransportServiceType-	transport service type, RC, UC, RD, UD
 *	Path				- 	Primary Path for connection
 *							data pointed to only used during duration of call
 *	AltPath				- 	Alternate Path for connection
 *							data pointed to only used during duration of call
 *
 * OUTPUTS:
 *	Info				- parameters to use to establish the connection.
 *						  and move the QP to QPStateInit
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *	FINVALID_PARAMETER
 *	FINSUFFICIENT_MEMORY
 *	FNOT_FOUND			- bad SGID or PKey in Path/AltPath
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *
 *	This routine validates the SGID and PKey in Path and AltPath
 *	it also confirms the Path and AltPath SGIDs are both from the same CA
 *	if this returns FNOT_FOUND, the caller should consider using different
 *	Path/AltPath values and try again.
 *
 * The request is partially built. The following must be filled in:
 * SID
 * CEPInfo.QPN - RC/UC QPN
 * CEPInfo.Qkey, LocalEECN, RemoteEECN - RD info
 *
 * The following are defaulted and may be adjusted by the caller:
 * OfferedInitiatorDepth - set to max for CA
 * OfferedResponderResources - set to max for CA
 * AckTimeout/AlternateAckTimeout - computed using LocalCaAckDelay and
 *				PktLifeTime, selected applications may need to increase
 *				this value
 * RetryCount - defaults to a reasonable value, appl may adjust down or up
 * RnRRetryCount - defaults to infinite
 * PrivateData - zeroed
 *
 * Per the IBTA spec, the QP should be moved to Init prior to calling
 * iba_cm_connect/iba_cm_connect_peer.
 *
 * The application must complete the QpAttrInit as follows:
 * QpAttrInit.AccessControl = desired settings
 * QpAttrInit.Attrs |= IB_QP_ATTR_ACCESSCONTROL
 */
typedef struct _CM_PREPARE_REQUEST_INFO {
	CM_REQUEST_INFO Request;			/* for iba_cm_connect[_peer] */
	IB_QP_ATTRIBUTES_MODIFY QpAttrInit;	/* to move QP to Init */
} CM_PREPARE_REQUEST_INFO;
	
typedef FSTATUS
(CM_PREPARE_REQUEST)(
	IN CM_CEP_TYPE TransportServiceType,
	IN const IB_PATH_RECORD *Path,
	IN const IB_PATH_RECORD *AltPath OPTIONAL,
	OUT CM_PREPARE_REQUEST_INFO *Info
	);
IBA_API CM_PREPARE_REQUEST iba_cm_prepare_request;

/*
 * iba_cm_connect/CmConnect
 *
 *	This routine is used by a client to establish a connection to the remote 
 *	server endpoint.
 *
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object that was returned in iba_cm_create_cep().
 *
 *	pConnectRequest	- Specify the parameters to use to establish the connection.
 *					  data pointed to only used during duration of call
 *
 *	pfnConnectCB	- Connection callback function. The CM notify the caller via this
 *					callback when a connect reply, reject or timeout occur.
 *
 *	Context			- This context is passed back in the callback routine.
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FPENDING
 *
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *	FCM_ADDR_INUSE
 *
 */
typedef FSTATUS
(CM_CONNECT)(
	IN IB_HANDLE				hCEP,
	IN const CM_REQUEST_INFO*	pConnectRequest,		/* Send REQ */
	IN PFN_CM_CALLBACK			pfnConnectCB,			/* Recv REP, REJ */
	IN void*					Context
	);
IBA_API CM_CONNECT iba_cm_connect;
IBA_API CM_CONNECT CmConnect;	/* for backward compatibility */

/*
 * iba_cm_connect_peer/CmConnectPeer
 *
 *	This routine is used by a peer to establish a connection to another remote 
 *	peer endpoint.
 *
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object that was returned in iba_cm_create_cep().
 *
 *	pPeerRequest	- Specify the parameters to use to establish the connection.
 *					  data pointed to only used during duration of call
 *
 *	pfnPeerCB		- Peer connection callback function. The CM notify the caller via this
 *					callback when a connect reply, reject or timeout occur.
 *
 *	Context			- This context is passed back in the callback routine.
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FPENDING
 *
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *	FCM_ADDR_INUSE
 *
 */
typedef FSTATUS
(CM_CONNECT_PEER)(
	IN IB_HANDLE				hCEP,
	IN const CM_REQUEST_INFO*	pPeerRequest,			/* Send REQ */
	IN PFN_CM_CALLBACK			pfnPeerCB,				/* Recv REP, REJ */
	IN void*					Context
	);
IBA_API CM_CONNECT_PEER iba_cm_connect_peer;
IBA_API CM_CONNECT_PEER CmConnectPeer;	/* for backward compatibility */


/*
 * iba_cm_listen/CmListen
 *
 *	This routine is used by a server to listen for incoming clients connections 
 *	on the specified endpoint.
 *
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object that was returned in iba_cm_create_cep().
 *
 *	pListenRequest	- Specify the parameters to listen for incoming connections on this
 *					endpoint.
 *					data pointed to only used during duration of call
 *
 *	pfnListenCB		- Listen callback function. The CM notify the caller via this
 *					callback when an incoming connection arrives on this endpoint.
 *
 *	Context			- This context is passed back in the callback routine.
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FPENDING
 *
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *	FCM_ADDR_INUSE
 *
 */
typedef FSTATUS
(CM_LISTEN)(
	IN IB_HANDLE				hCEP,
	IN const CM_LISTEN_INFO*	pListenRequest,
	IN PFN_CM_CALLBACK			pfnListenCB,			/* Recv REQ */
	IN void*					Context
	);
IBA_API CM_LISTEN iba_cm_listen;
IBA_API CM_LISTEN CmListen;	/* for backward compatibility */



/*
 * iba_cm_cancel/CmCancel
 *
 *	This routine cancels a pending connect or listen request.
 *
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object used in iba_cm_connect()
 *						or iba_cm_listen().
 *
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FPENDING
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 */
typedef FSTATUS
(CM_CANCEL)(
	IN IB_HANDLE				hCEP
	);
IBA_API CM_CANCEL iba_cm_cancel;
IBA_API CM_CANCEL CmCancel;	/* for backward compatibility */


/*
 * iba_cm_process_request
 *
 *	Perform parameter negotiation and help build the Reply for a 
 *	server which is processing an inbound FCM_CONNECT_REQUEST
 *
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object reported by FCM_CONNECT_REQUEST
 *	Failover		- FailoverAccepted value desired in reply
 *						(ignored if failover is	not supported by CEP or CA)
 *
 *
 * OUTPUTS:
 *
 * 	Info			- Information required to move QP to Init and RTR
 * 						and majority of Reply for use in iba_cm_accept
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FINSUFFICIENT_MEMORY
 *	FNOT_FOUND
 *	FCM_CONNECT_CANCEL
 *
 * The reply is partially built. The following must be filled in:
 * Reply.QPN - RC/UC QPN
 * Reply.Qkey, EECN - RD info
 *
 * The following are defaulted and may be adjusted by the caller:
 * ArbInitiatorDepth - negotiated based on max for CA
 * ArbResponderResources - negotiated based on max for CA
 * RnRRetryCount - defaults to value used by client side
 * PrivateData - zeroed
 *
 * The application must complete the QpAttrInit as follows:
 * QpAttrInit.AccessControl = desired settings
 * QpAttrInit.Attrs |= IB_QP_ATTR_ACCESSCONTROL
 * 
 * The application must complete the QpAttrRtr as follows:
 * QpAttrRtr.MinRnrTimer = UsecToRnrNakTimer(appl desired value)
 * QpAttrRtr.Attrs |= IB_QP_ATTR_MINRNRTIMER
 */

typedef struct _CM_PROCESS_REQUEST_INFO {
		CM_CONN_INFO 			ReplyInfo;
		IB_QP_ATTRIBUTES_MODIFY QpAttrInit;
		IB_QP_ATTRIBUTES_MODIFY QpAttrRtr;
} CM_PROCESS_REQUEST_INFO;

typedef FSTATUS
(CM_PROCESS_REQUEST)(
	IN IB_HANDLE 				hCEP,
	IN CM_REP_FAILOVER			Failover,
	OUT CM_PROCESS_REQUEST_INFO*Info
	);
IBA_API CM_PROCESS_REQUEST iba_cm_process_request;

/*
 * iba_cm_accept/CmAccept
 *
 *	This routine is used by a client to accept the connect reply from the server
 *	and set this endpoint into a connected state. This routine is used by a server to
 *	accept and reply to the client's connect request.  
 *
 *
 * INPUTS: (Server)
 *
 *	hCEP			- Handle to the CEP object. This handle must be the same
 *						handle passed to iba_cm_listen().
 *
 *	pSendConnInfo	- Specify the reply info to send back to the remote client endpoint.
 *					  data pointed to only used during duration of call
 *
 *	pRecvConnInfo	- Specify the returned rtu info from the remote client endpoint.
 *					endpoint. (if CM_FLAG_ASYNC_ACCEPT is set for CEP, this is
 *					not used)
 *					data pointed to only used during duration of call
 *
 *	pfnCallback 	- Callback function to use for future events on this
 *					CEP (namely Disconnect, APM or for Async Accept, the
 *					connection 	establishment/failure).
 *					Ignored for Peer to Peer.
 *
 *	Context			- context for new CEP.  not used for peer to peer.
 *
 * OUTPUTS: (Server)
 *
 *	hNewCEP			- Handle to the new CEP object that represent the new connection
 *					from client. (For peer to peer connections this is optional
 *					if provided nCEP will be returned here)
 *					When using Async Accept, the server must be ready for
 *					callbacks to occur immediately after iba_cm_accept.  As such
 *					it is recommended that the pointer given here be the actual
 *					storage area for the new CEP handle (as opposed to a local
 *					temp variable).
 *
 *
 *
 * INPUTS: (Client)
 *
 *	hCEP			- Handle to the CEP object. This handle must be the same
 *						handle passed to iba_cm_connect().
 *
 *	pSendConnInfo	- Specify the rtu info to send back to the remote server endpoint.
 *					  If NULL a default (zero PrivateData) RTU will be sent
 *
 *	pRecvConnInfo	- Not used.
 *
 *	pfnCallback		- Not used.
 *
 *	Context			- Not used.
 *
 *
 * OUTPUTS: (Client)
 *
 *	hNewCEP			- Not used.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS - Client RTU queued to be sent.
 *	FCM_CONNECT_ESTABLISHED - server REP sent and RTU received
 *	FCM_CONNECT_REJECT - server REP sent and REJ received
 *	FCM_CONNECT_TIMEOUT - server REP sent and timeout waiting for RTU
 *	FCM_CONNECT_CANCEL - listen on this CEP has been canceled
 *
 *	FERROR	- Unable to send the reply ack packet
 *	FTIMEOUT - The timeout interval expires
 *	FINVALID_STATE - The endpoint is not in the valid state for this call
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *	FPENDING - Server Async Accept, REP queued to be sent.
 *
 *
 *
 * IRQL:
 *
 */
typedef FSTATUS
(CM_ACCEPT)(
	IN IB_HANDLE			hCEP,
	IN CM_CONN_INFO*		pSendConnInfo,				/* Send REP, Send RTU */
	OUT CM_CONN_INFO*		pRecvConnInfo,				/* Recv RTU, Recv REJ */
	IN PFN_CM_CALLBACK		pfnCallback,				/* For hNewCEP */
	IN void*				Context,					/* For hNewCEP */
	OUT IB_HANDLE*			hNewCEP
	);
IBA_API CM_ACCEPT iba_cm_accept;
IBA_API CM_ACCEPT CmAccept;	/* for backward compatibility */


/*
 * iba_cm_reject/CmReject
 *
 *	This routine is used by a client to reject the connect reply from the server .
 *	This routine is also used by a server to reject the incoming client's connect request.
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object.
 *
 *	pReject			- Specify the reject info that will be send back to the remote endpoint.
 *					  data pointed to only used during duration of call
 *
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FERROR
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *
 */
typedef FSTATUS
(CM_REJECT)(
	IN IB_HANDLE				hCEP,
	IN const CM_REJECT_INFO*	pRejectInfo				/* Send REJ */
	);
IBA_API CM_REJECT iba_cm_reject;
IBA_API CM_REJECT CmReject;	/* for backward compatibility */

/*
 * iba_cm_process_reply
 *
 *	This routine is used by a client to compose the QP Attributes to
 *	move to QPStateReadyToSend 
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object.
 *
 *
 * OUTPUTS:
 *
 *	Info			- QP Attributes which should be used to move to RTS
 *					  data pointed to only used during duration of call
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FERROR
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *
 *
 * The application must complete the QpAttrRtr as follows:
 * QpAttrRtr.MinRnrTimer = UsecToRnrNakTimer(appl desired value)
 * QpAttrRtr.Attrs |= IB_QP_ATTR_MINRNRTIMER
 *
 * The complete set of QpAttrRts attributes is generated.
 */
typedef struct _CM_PROCESS_REPLY_INFO {
		IB_QP_ATTRIBUTES_MODIFY QpAttrRtr;
		IB_QP_ATTRIBUTES_MODIFY QpAttrRts;
} CM_PROCESS_REPLY_INFO;

typedef FSTATUS
(CM_PROCESS_REPLY)(
	IN IB_HANDLE			hCEP,
	OUT CM_PROCESS_REPLY_INFO *Info
	);
IBA_API CM_PROCESS_REPLY iba_cm_process_reply;


/*
 * iba_cm_prepare_rts
 *
 *	This routine is used by a server to compose the QP Attributes to
 *	move to QPStateReadyToSend
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object.
 *
 *
 * OUTPUTS:
 *
 *	QpAttrRts		- QP Attributes which should be used to move to RTS
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FERROR
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *
 *
 * The completed set of required attributes is generated.
 * If desired the application may add optional parameters
 * such as MinRnrTimer, SendQ/RecvQDepth SendDSListDepth/RecvDSListDepth
 */
typedef FSTATUS
(CM_PREPARE_RTS)(
	IN IB_HANDLE			hCEP,
	OUT IB_QP_ATTRIBUTES_MODIFY *QpAttrRts
	);
IBA_API CM_PREPARE_RTS iba_cm_prepare_rts;

/*
 * iba_cm_altpath_request/CmAltPathRequest
 *
 * Request a new alternate path
 * Only allowed for Client/Active side of connection
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object.
 *	pLapInfo		- Information used to build the LAP to send
 *						if AlternateAckTimeout is 0, CM will
 *						compute an appropriate value based on
 *						PktLifeTime for the path and the LocalCaAckDelay
 *					    data pointed to only used during duration of call
 *
 *
 * OUTPUTS:
 *
 *	None.
 *
 * RETURNS:
 *
 *	FPENDING - LAP queued to be sent.
 *	FINVALID_STATE - The endpoint is not in the valid state for this call
 *	FERROR	- Unable to send the lap packet
 *	FINVALID_ARGUMENTS - invalid Info.Lap
 *
 *
 * IRQL:
 *
 *	This routine is called at IRQL_PASSIVE_LEVEL.
 */
typedef FSTATUS 
(CM_ALTPATH_REQUEST)(
	IN IB_HANDLE				hCEP,
	IN const CM_ALTPATH_INFO*	pLapInfo		/* Send LAP */
	);
IBA_API CM_ALTPATH_REQUEST iba_cm_altpath_request;

/*
 * iba_cm_process_altpath_request
 *
 * process a FCM_ALTPATH_REQUEST.  This prepares the QP Attributes
 * to set the new alternate path and prepares a Reply
 * Only allowed for Server/Passive side of connection
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object.
 *	AltPathRequest	- AltPath Request we received
 *					  data pointed to only used during duration of call
 *
 *
 * OUTPUTS:
 *
 *	Info			- constructed QP Attributes and Alt Path Reply
 *
 * RETURNS:
 *
 *	FINVALID_STATE - The endpoint is not in the valid state for this call
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *
 *
 * IRQL:
 *
 *	This routine is called at IRQL_PASSIVE_LEVEL.
 *
 * The completed set of required attributes is generated.
 *
 * The reply which is prepared is a simple accept of the alternate path
 * The following are defaulted and may be adjusted by the caller:
 * PrivateData - zeroed
 */
typedef struct _CM_PROCESS_ALTPATH_REQUEST_INFO {
		CM_ALTPATH_REPLY_INFO AltPathReply;
		IB_QP_ATTRIBUTES_MODIFY QpAttrRts;
} CM_PROCESS_ALTPATH_REQUEST_INFO;

typedef FSTATUS 
(CM_PROCESS_ALTPATH_REQUEST)(
	IN IB_HANDLE				hCEP,
	IN CM_ALTPATH_INFO*			AltPathRequest,
	OUT CM_PROCESS_ALTPATH_REQUEST_INFO* Info
	);
IBA_API CM_PROCESS_ALTPATH_REQUEST iba_cm_process_altpath_request;

/*
 * iba_cm_altpath_reply/CmAltPathReply
 *
 * Reply to an alternate path request
 * Only allowed for Server/Passive side of connection
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object.
 *	pAprInfo		- AltPath Reply
 *					  data pointed to only used during duration of call
 *
 *
 * OUTPUTS:
 *
 *	None.
 *
 * RETURNS:
 *
 *	FSUCCESS - APR queued to be sent.
 *	FINVALID_STATE - The endpoint is not in the valid state for this call
 *	FERROR	- Unable to send the apr packet
 *
 *
 * IRQL:
 *
 *	This routine is called at IRQL_PASSIVE_LEVEL.
 */
typedef FSTATUS 
(CM_ALTPATH_REPLY)(
	IN IB_HANDLE				hCEP,
	IN CM_ALTPATH_REPLY_INFO*	pAprInfo		/* Send APR */
	);
IBA_API CM_ALTPATH_REPLY iba_cm_altpath_reply;


/*
 * iba_cm_process_altpath_reply
 *
 * process a FCM_ALTPATH_REPLY.  This prepares the QP Attributes
 * to set the new alternate path
 * Only allowed for Client/Active side of connection
 *
 * INPUTS:
 *
 *	hCEP			- Handle to the CEP object.
 *	AltPathReply	- AltPath Reply we received
 *					  data pointed to only used during duration of call
 *
 *
 * OUTPUTS:
 *
 *	QpAttrRts		- constructed QP Attributes
 *
 * RETURNS:
 *
 *	FINVALID_STATE - The endpoint is not in the valid state for this call
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *
 *
 * IRQL:
 *
 *	This routine is called at IRQL_PASSIVE_LEVEL.
 *
 * The completed set of required attributes is generated.
 */
typedef FSTATUS 
(CM_PROCESS_ALTPATH_REPLY)(
	IN IB_HANDLE					hCEP,
	IN const CM_ALTPATH_REPLY_INFO*	AltPathReply,
	OUT IB_QP_ATTRIBUTES_MODIFY*	QpAttrRts
	);
IBA_API CM_PROCESS_ALTPATH_REPLY iba_cm_process_altpath_reply;

/*
 * iba_cm_migrated/CmMigrated
 *
 * Inform CM that alternate path has been migrated to
 * FUTURE for server side may be hooked into CM Async Event callbacks
 *
 * INPUTS:
 *
 *
 *
 * OUTPUTS:
 *
 *	None.
 *
 * RETURNS:
 *
 *	FSUCCESS - CEP adjusted to reflect migration
 *	FINVALID_STATE - The endpoint is not in the valid state for this call
 *
 *
 * IRQL:
 *
 *	This routine is called at IRQL_PASSIVE_LEVEL.
 */
typedef FSTATUS 
(CM_MIGRATED)(
	IN IB_HANDLE				hCEP
	);
IBA_API CM_MIGRATED iba_cm_migrated;


/*
 * iba_cm_migrated_reload
 *
 * Inform CM that alternate path has been migrated to
 * and prepare a AltPath Request to load previous primary path
 * as the new alternate path
 * also returns new primary path
 *
 * This is useful is an application wants a simple dual path APM
 * however this can be risky in that it assumes the old primary path will
 * be restored as it was (which may not be the case if cables are moved
 * or fabric is otherwise reconfigured to restore the old path)
 * Alternately the AltPathRequest can also be used to simply resolve a new path
 * to the old primary DGID and then update its Path before doing the
 * iba_cm_altpath_request.  The NewPrimaryPathInfo is also provided
 * so the caller can select a path which is distinct from the New Primary Path
 *
 * This is a superset of the iba_cm_migrated
 *
 * INPUTS:
 *
 *
 *
 * OUTPUTS:
 *
 *	NewPrimaryPath			- new Primary Path (after migration)
 *							AckTimeout will be accurate
 *							however the PktLifeTime may not be
 *	AltPathRequest			- a AltPath Request representing the old primary
 *							path.  The Alternate AckTimeout will be accurate
 *							however the PktLifeTime may not be
 *
 * RETURNS:
 *
 *	FSUCCESS - CEP adjusted to reflect migration
 *	FINVALID_STATE - The endpoint is not in the valid state for this call
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *
 *
 * IRQL:
 *
 *	This routine is called at IRQL_PASSIVE_LEVEL.
 */

typedef struct _CM_PATH_INFO {
	CM_CEP_PATHINFO	PathInfo;
	uint8			AckTimeout;	
} CM_PATH_INFO;

typedef FSTATUS 
(CM_MIGRATED_RELOAD)(
	IN IB_HANDLE				hCEP,
	OUT CM_PATH_INFO*			NewPrimaryPath OPTIONAL,
	OUT CM_ALTPATH_INFO*		AltPathRequest OPTIONAL
	);
IBA_API CM_MIGRATED_RELOAD iba_cm_migrated_reload;

/*
 * iba_cm_wait/CmWait
 *
 * For an application which wants to be single threaded, this can be used
 * to wait for the next event on a set of CEPs and obtain the Callback info
 * which would have been provided for any events on those CEPs.
 *	This routine is used by a caller to synchronous with its previous asynchronous operations without
 *	using the callback notification. The iba_cm_connect(), iba_cm_listen()
 *  and iba_cm_disconnect() are asynchronous in nature.
 *
 * INPUTS:
 *
 *	CEPHandleArray[]- An array of handle to the CEP object to wait on.
 *					  data pointed to only used during duration of call
 *
 *	ConnInfoArray[]	- An array of connection info buffers.
 *					  data pointed to only used during duration of call
 *
 *	ArrayCount		- Number of CEP handle in CEPHandleArray[].
 *
 *	Timeout_us		- Timeout interval.
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FERROR
 *	FTIMEOUT
 */
typedef FSTATUS
(CM_WAIT)(
	IN IB_HANDLE			CEPHandleArray[],
	OUT CM_CONN_INFO*		ConnInfoArray[],
	IN uint32				ArrayCount,
	IN uint32				Timeout_us
	);
IBA_API CM_WAIT iba_cm_wait;
IBA_API CM_WAIT CmWait;	/* for backward compatibility */


/*
 * iba_cm_disconnect/CmDisconnect
 *
 *	This routine is used by a client or a server to close the current connection
 *	on the specified endpoint.
 *
 * INPUTS:
 *
 *	hCEP				- Handle to the CEP object.
 *
 *	pDisconnectRequest	- Specify the disconnect request. This request is send to the remote endpoint
 *						to inform it that this endpoint is closing down.
 *					    data pointed to only used during duration of call
 *
 *	pDisconnectReply	- Specify the disconnect reply to the disconnect request. Caller call 
 *						optionally this routine and passes in a reply to response to the  
 *						disconnect notification.
 *					    data pointed to only used during duration of call
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FERROR
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *	FCM_ALREADY_DISCONNECTING
 *
 */
typedef FSTATUS
(CM_DISCONNECT)(
	IN IB_HANDLE				hCEP,
	IN const CM_DREQUEST_INFO*	pDisconnectRequest,	/* Send DREQ */
	IN const CM_DREPLY_INFO*	pDisconnectReply	/* Send DREP */
	);
IBA_API CM_DISCONNECT iba_cm_disconnect;
IBA_API CM_DISCONNECT CmDisconnect;	/* for backward compatibility */


/*
 * iba_cm_set_wait_obj/CmSetWaitObj
 *
 *	Associate the specified endpoint with the event wait obj.
 *  This is useful for applications which want to implement their own
 *  synchronization mechanism based on their own event and iba_cm_wait instead
 *  of direct Cm callbacks.
 *
 *	Not yet implemented
 *
 * INPUTS:
 *
 *	hCEP				- Handle to the CEP object.
 *
 *	hWaitEvent			- Event wait handle.
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FERROR
 */
#if 0
typedef FSTATUS
(CM_SET_WAIT_OBJ)(
	IN IB_HANDLE				hCEP,
	IN EVENT_HANDLE				hWaitEvent
	);
IBA_API CM_SET_WAIT_OBJ iba_cm_set_wait_obj;
IBA_API CM_SET_WAIT_OBJ CmSetWaitObj;	/* for backward compatibility */
#endif

/*
 * UD Service ID resolution routines
 *

 *
 * iba_cm_sidr_register/SIDRRegister
 *
 *	This routine is used by a UD server to register its Service ID (SID) for
 *	the specified endpoint so that UD client can lookup the QPN the UD server is using.
 *
 * INPUTS:
 *
 *	hCEP				- Handle to the CEP object. This is the handle returned from iba_cm_create_cep().
 *
 *	pSIDRRegisterInfo	- Registeration info.
 *					      data pointed to only used during duration of call
 *
 *	pfnQueryCallback	- Query callback function. The CM notify the caller via this callback
 *						when an incoming SIDR query is made on this endpoint. This allows
 *						the UD server to "trap" the queries.
 *
 *	Context				- This context field is passed back in the callback routine.
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 *	FCM_ADDR_INUSE
 */
typedef FSTATUS
(CM_SIDR_REGISTER)(
	IN IB_HANDLE				hCEP,
	IN const SIDR_REGISTER_INFO* pSIDRRegisterInfo,
	IN PFN_CM_CALLBACK			pfnQueryCallback,
	IN void*					Context
	);
IBA_API CM_SIDR_REGISTER iba_cm_sidr_register;
IBA_API CM_SIDR_REGISTER SIDRRegister;	/* for backward compatibility */

/*
 * iba_cm_sidr_deregister/SIDRDeregister
 *
 *	Deregister the UD server's SID.
 *
 * INPUTS:
 *
 *	hCEP				- Handle to the CEP object. This is the handle that
 *							was passed to iba_cm_sidr_register().
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 */
typedef FSTATUS
(CM_SIDR_DEREGISTER)(
	IN IB_HANDLE	hCEP
	);
IBA_API CM_SIDR_DEREGISTER iba_cm_sidr_deregister;
IBA_API CM_SIDR_DEREGISTER SIDRDeregister;	/* for backward compatibility */


/*
 * iba_cm_sidr_response/SIDRResponse
 *
 *	The routine is called to response to a SIDR query.
 *
 * INPUTS:
 *
 *	hCEP				- Handle to the registered CEP object.
 *
 *	pSIDRResponse		- SIDR response
 *					      data pointed to only used during duration of call
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 */
typedef FSTATUS
(CM_SIDR_RESPONSE)(
	IN IB_HANDLE			hCEP,
	IN const SIDR_RESP_INFO* pSIDRResponse		/* Send SIDR_RESP */
	);
IBA_API CM_SIDR_RESPONSE iba_cm_sidr_response;
IBA_API CM_SIDR_RESPONSE SIDRResponse;	/* for backward compatibility */



/*
 * iba_cm_sidr_query/SIDRQuery
 *
 *	The routine is called to perform a SIDR query.
 *
 * INPUTS:
 *
 *	hCEP				- Handle to the CEP object.
 *
 *	pSIDRRequest		- SIDR request
 *					      data pointed to only used during duration of call
 *
 *	pfnQueryCallback	-
 *
 *	Context				- This context field is passed back in the callback routine.
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *
 *	FINVALID_STATE
 *	FINVALID_PARAMETER
 *	FCM_NOT_INITIALIZED
 *	FCM_INVALID_HANDLE
 */
typedef FSTATUS
(CM_SIDR_QUERY)(
	IN IB_HANDLE			hCEP,
	IN const SIDR_REQ_INFO*	pSIDRRequest,		/* Send SIDR_REQ */
	IN PFN_CM_CALLBACK		pfnQueryCallback,	/* Recv SIDR_RESP */
	IN void*				Context
	);
IBA_API CM_SIDR_QUERY iba_cm_sidr_query;
IBA_API CM_SIDR_QUERY SIDRQuery;	/* for backward compatibility */

#if defined (__cplusplus)
};
#endif

#endif  /* _IBA_IB_CM_H_ */
