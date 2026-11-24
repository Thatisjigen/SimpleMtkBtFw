//
//  SimpleMtkBtPatcher.cpp
//  SimpleMtkBtFw
//
//  Created by laobamac on 2026/5/30.
//  Copyright © 2026 laobamac. All rights reserved.
//

//
//  SimpleMtkBtPatcher.cpp
//  SimpleMtkBtPatcher
//
//  Created by zxystd <zxystd@foxmail.com> on 2021/2/8.
//

#include <Headers/kern_api.hpp>
#include <Headers/kern_util.hpp>
#include <Headers/plugin_start.hpp>

#include "SimpleMtkBtPatcher.hpp"

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOMessage.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <sys/sysctl.h>
#include <stdarg.h>

static CSimpleMtkBtPatcher mtkBtPatcher;
static CSimpleMtkBtPatcher *callbackMtkBtPatcher = nullptr;

static void sleepInit();
static void sleepNoteDevice();
static void sleepNoteCommand(void *device, IOService *client, const StandardUSB::DeviceRequest &request, uint16_t opcode);
static void sleepParseAcl(const uint8_t *b, uint32_t have, uint32_t total, bool in);
static IOReturn sleepWatchIn(void *that, IOMemoryDescriptor *buffer, uint32_t length, IOUSBHostCompletion *completion, uint32_t timeoutMs, bool events);

static const char *bootargOff[] {
    "-smbtcompatoff"
};

static const char *bootargDebug[] {
    "-smbtcompatdbg"
};

static const char *bootargBeta[] {
    "-smbtcompatbeta"
};

PluginConfiguration ADDPR(config) {
    xStringify(PRODUCT_NAME),
    parseModuleVersion(xStringify(MODULE_VERSION)),
    LiluAPI::AllowNormal | LiluAPI::AllowInstallerRecovery | LiluAPI::AllowSafeMode,
    bootargOff,
    arrsize(bootargOff),
    bootargDebug,
    arrsize(bootargDebug),
    bootargBeta,
    arrsize(bootargBeta),
    KernelVersion::MountainLion,
    KernelVersion::Tahoe,
    []() {
        mtkBtPatcher.init();
    }
};

static const char *SimpleMtkBtPatcher_IOBluetoothFamily[] { "/System/Library/Extensions/IOBluetoothFamily.kext/Contents/MacOS/IOBluetoothFamily" };

static KernelPatcher::KextInfo SimpleMtkBtPatcher_IOBluetoothInfo {
    "com.apple.iokit.IOBluetoothFamily",
    SimpleMtkBtPatcher_IOBluetoothFamily,
    1,
    {true, true},
    {},
    KernelPatcher::KextInfo::Unloaded
};

static const char *SimpleMtkBtPatcher_IOUSBHostFamily[] {
    "/System/Library/Extensions/IOUSBHostFamily.kext/Contents/MacOS/IOUSBHostFamily" };

static KernelPatcher::KextInfo SimpleMtkBtPatcher_IOUsbHostInfo {
    "com.apple.iokit.IOUSBHostFamily",
    SimpleMtkBtPatcher_IOUSBHostFamily,
    1,
    {true, true},
    {},
    KernelPatcher::KextInfo::Unloaded
};

bool CSimpleMtkBtPatcher::_randomAddressInit = false;
bool CSimpleMtkBtPatcher::_randomAddressHack = false;
tUSBHostDeviceAddress CSimpleMtkBtPatcher::_mtkAddress = 0;
bool CSimpleMtkBtPatcher::_mtkAddressValid = false;

bool CSimpleMtkBtPatcher::init()
{
    DBGLOG(DRV_NAME, "%s", __PRETTY_FUNCTION__);
    callbackMtkBtPatcher = this;
    sleepInit();
    if (getKernelVersion() < KernelVersion::Monterey) {
        lilu.onKextLoadForce(&SimpleMtkBtPatcher_IOBluetoothInfo, 1,
        [](void *user, KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
            callbackMtkBtPatcher->processKext(patcher, index, address, size);
        }, this);
    } else {
        lilu.onKextLoadForce(&SimpleMtkBtPatcher_IOUsbHostInfo, 1,
        [](void *user, KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
            callbackMtkBtPatcher->processKext(patcher, index, address, size);
        }, this);
    }
    return true;
}

void CSimpleMtkBtPatcher::free()
{
    DBGLOG(DRV_NAME, "%s", __PRETTY_FUNCTION__);
}

void CSimpleMtkBtPatcher::processKext(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size)
{
    DBGLOG(DRV_NAME, "%s", __PRETTY_FUNCTION__);
    if (getKernelVersion() < KernelVersion::Monterey) {
        if (SimpleMtkBtPatcher_IOBluetoothInfo.loadIndex == index) {
            DBGLOG(DRV_NAME, "%s", SimpleMtkBtPatcher_IOBluetoothInfo.id);
            
            KernelPatcher::RouteRequest findQueueRequestRequest {
                "__ZN25IOBluetoothHostController17FindQueuedRequestEtP22BluetoothDeviceAddresstbPP21IOBluetoothHCIRequest",
                newFindQueueRequest,
                oldFindQueueRequest
            };
            patcher.routeMultiple(index, &findQueueRequestRequest, 1, address, size);
            if (patcher.getError() == KernelPatcher::Error::NoError) {
                DBGLOG(DRV_NAME, "routed %s", findQueueRequestRequest.symbol);
            } else {
                SYSLOG(DRV_NAME, "failed to resolve %s, error = %d", findQueueRequestRequest.symbol, patcher.getError());
                patcher.clearError();
            }
            
        }
    } else {
        if (SimpleMtkBtPatcher_IOUsbHostInfo.loadIndex == index) {
            SYSLOG(DRV_NAME, "%s", SimpleMtkBtPatcher_IOUsbHostInfo.id);

            KernelPatcher::RouteRequest pipeIoRequest {
                "__ZN13IOUSBHostPipe2ioEP18IOMemoryDescriptorjP19IOUSBHostCompletionj",
                newPipeIo,
                oldPipeIo
            };
            patcher.routeMultiple(index, &pipeIoRequest, 1, address, size);
            if (patcher.getError() == KernelPatcher::Error::NoError) {
                SYSLOG(DRV_NAME, "routed %s", pipeIoRequest.symbol);
            } else {
                SYSLOG(DRV_NAME, "failed to resolve %s, error = %d", pipeIoRequest.symbol, patcher.getError());
                patcher.clearError();
            }

            /*
             * deviceRequest carries HCI commands (EP0). It is hooked for the
             * Read Clock fix; the LE random address workaround (written for
             * Intel controllers) stays opt-in via -smbtrandaddr.
             */
            _randomAddressHack = checkKernelArgument("-smbtrandaddr");

            KernelPatcher::RouteRequest hostDeviceRequest {
            "__ZN15IOUSBHostDevice13deviceRequestEP9IOServiceRN11StandardUSB13DeviceRequestEPvP18IOMemoryDescriptorRjP19IOUSBHostCompletionj",
                newHostDeviceRequest,
                oldHostDeviceRequest
            };
            patcher.routeMultiple(index, &hostDeviceRequest, 1, address, size);
            if (patcher.getError() == KernelPatcher::Error::NoError) {
                SYSLOG(DRV_NAME, "routed %s", hostDeviceRequest.symbol);
            } else {
                SYSLOG(DRV_NAME, "failed to resolve %s, error = %d", hostDeviceRequest.symbol, patcher.getError());
                patcher.clearError();
            }
        }
    }
}

#pragma mark - For Bigsur-, patch unhandled 0x2019 opcode

#define HCI_OP_LE_START_ENCRYPTION 0x2019

IOReturn CSimpleMtkBtPatcher::newFindQueueRequest(void *that, unsigned short arg1, void *addr, unsigned short arg2, bool arg3, void **hciRequestPtr)
{
    IOReturn ret = FunctionCast(newFindQueueRequest, callbackMtkBtPatcher->oldFindQueueRequest)(that, arg1, addr, arg2, arg3, hciRequestPtr);
    if (ret != 0 && arg1 == HCI_OP_LE_START_ENCRYPTION) {
        ret = FunctionCast(newFindQueueRequest, callbackMtkBtPatcher->oldFindQueueRequest)(that, arg1, addr, 0xffff, arg3, hciRequestPtr);
        DBGLOG(DRV_NAME, "%s ret: %d arg1: 0x%04x arg2: 0x%04x arg3: %d ptr: %p", __FUNCTION__, ret, arg1, arg2, arg3, *hciRequestPtr);
    }
    return ret;
}

#pragma mark - For Monterey+ patch for intercept HCI REQ and RESP

StandardUSB::DeviceRequest randomAddressRequest;
// Hardcoded Random Address HCI Command
const uint8_t randomAddressHci[9] = {0x05, 0x20, 0x06, 0x94, 0x50, 0x64, 0xD0, 0x78, 0x6B}; 
IOBufferMemoryDescriptor *writeHCIDescriptor = nullptr;

#define MAX_HCI_BUF_LEN             255
#define HCI_OP_RESET                0x0c03
#define HCI_OP_LE_SET_SCAN_PARAM    0x200B
#define HCI_OP_LE_SET_SCAN_ENABLE   0x200C

#define MTK_BT_VENDOR_ID  0x13D3
#define MTK_BT_PRODUCT_ID 0x3622

/*
 * The deviceRequest and pipe io hooks are installed on methods that every USB
 * device goes through, but only the MediaTek controller (13D3:3622) must be
 * touched: its HCI Read Clock reply is rewritten, and its bulk OUT ACL header
 * flag is rewritten. Match the controller by the idVendor/idProduct registry
 * properties, which need no USB I/O, and remember its bus address so the pipe
 * hook -- which is only handed a pipe -- can tell the controller's transfers
 * from every other device's.
 *
 * Do not call getDeviceDescriptor() here: deviceRequest also carries the
 * requests that read that descriptor while a device enumerates, and asking for
 * it from inside one of them leaves every USB device unenumerated.
 */
bool CSimpleMtkBtPatcher::isMtkBtDevice(void *device)
{
    IOUSBHostDevice *dev = OSDynamicCast(IOUSBHostDevice, static_cast<OSObject *>(device));
    if (dev == nullptr)
        return false;
    OSNumber *vid = OSDynamicCast(OSNumber, dev->getProperty("idVendor"));
    OSNumber *pid = OSDynamicCast(OSNumber, dev->getProperty("idProduct"));
    bool match = vid != nullptr && pid != nullptr &&
        vid->unsigned16BitValue() == MTK_BT_VENDOR_ID && pid->unsigned16BitValue() == MTK_BT_PRODUCT_ID;
    if (match) {
        _mtkAddress = dev->getAddress();
        _mtkAddressValid = true;
        sleepNoteDevice();
    } else if (_mtkAddressValid && dev->getAddress() == _mtkAddress) {
        /* the controller's old bus address now belongs to another device */
        _mtkAddressValid = false;
    }
    return match;
}

IOReturn CSimpleMtkBtPatcher::newHostDeviceRequest(void *that, IOService *provider, StandardUSB::DeviceRequest &request, void *data, IOMemoryDescriptor *descriptor, unsigned int &length, IOUSBHostCompletion *completion, unsigned int timeout)
{
    HciCommandHdr *hdr = nullptr;
    uint32_t hdrLen = 0;
    char hciBuf[MAX_HCI_BUF_LEN] = {0};

    /*
     * Only host-to-device class requests can carry HCI commands. Everything
     * else, enumeration included, goes straight through before any check on
     * the device.
     */
    if ((request.bmRequestType & 0xe0) != 0x20 || !isMtkBtDevice(that))
        return FunctionCast(newHostDeviceRequest, callbackMtkBtPatcher->oldHostDeviceRequest)(that, provider, request, data, descriptor, length, completion, timeout);

    fixReadClock(request, data, descriptor);
    
    if (data == nullptr) {
        if (descriptor != nullptr &&
            (getKernelVersion() < KernelVersion::Sequoia || !descriptor->prepare(kIODirectionOut))) {
            if (descriptor->getLength() > 0) {
                descriptor->readBytes(0, hciBuf, min(descriptor->getLength(), MAX_HCI_BUF_LEN));
                hdrLen = (uint32_t)min(descriptor->getLength(), MAX_HCI_BUF_LEN);
            }
            if (getKernelVersion() >= KernelVersion::Sequoia)
                descriptor->complete(kIODirectionOut);
        }
        hdr = (HciCommandHdr *)hciBuf;
        if (_randomAddressHack && hdr->opcode == HCI_OP_LE_SET_SCAN_PARAM) {
            if (!_randomAddressInit) {
                randomAddressRequest.bmRequestType = makeDeviceRequestbmRequestType(kRequestDirectionOut, kRequestTypeClass, kRequestRecipientInterface);
                randomAddressRequest.bRequest = 0xE0;
                randomAddressRequest.wIndex = 0;
                randomAddressRequest.wValue = 0;
                randomAddressRequest.wLength = 9;
                length = 9;
                if (writeHCIDescriptor == nullptr)
                    writeHCIDescriptor = IOBufferMemoryDescriptor::withBytes(randomAddressHci, 9, kIODirectionOut);
                writeHCIDescriptor->prepare(kIODirectionOut);
                IOReturn ret = FunctionCast(newHostDeviceRequest, callbackMtkBtPatcher->oldHostDeviceRequest)(that, provider, randomAddressRequest, nullptr, writeHCIDescriptor, length, nullptr, timeout);
                writeHCIDescriptor->complete();
                const char *randAddressDump = _hexDumpHCIData((uint8_t *)randomAddressHci, 9);
                if (randAddressDump) {
                    SYSLOG(DRV_NAME, "[PATCH] Sending Random Address HCI %d %s", ret, randAddressDump);
                    IOFree((void *)randAddressDump, 9 * 3 + 1);
                }
                _randomAddressInit = true;
                SYSLOG(DRV_NAME, "[PATCH] Resend LE SCAN PARAM HCI %d", ret);
            }
        }
    } else {
        hdr = (HciCommandHdr *)data;
        hdrLen = request.wLength - 3;
    }
    if (hdr) {
        if (data != nullptr || hdrLen >= sizeof(HciCommandHdr))
            sleepNoteCommand(that, provider, request, hdr->opcode);
        // HCI reset, we need to send Random address again
        if (hdr->opcode == HCI_OP_RESET)
            _randomAddressInit = false;
#if DEBUG
        DBGLOG(DRV_NAME, "[%s] bRequest: 0x%x direction: %s type: %s recipient: %s wValue: 0x%02x wIndex: 0x%02x opcode: 0x%04x len: %d length: %d async: %d", provider->getName(), request.bRequest, requestDirectionNames[(request.bmRequestType & kDeviceRequestDirectionMask) >> kDeviceRequestDirectionPhase], requestRecipientNames[(request.bmRequestType & kDeviceRequestRecipientMask) >> kDeviceRequestRecipientPhase], requestTypeNames[(request.bmRequestType & kDeviceRequestTypeMask) >> kDeviceRequestTypePhase], request.wValue, request.wIndex, hdr->opcode, hdr->len, request.wLength, completion != nullptr);
        if (hdrLen) {
            const char *dump = _hexDumpHCIData((uint8_t *)hdr, hdrLen);
            if (dump) {
                DBGLOG(DRV_NAME, "[Request]: %s", dump);
                IOFree((void *)dump, hdrLen * 3 + 1);
            }
        }
#endif
    }
    return FunctionCast(newHostDeviceRequest, callbackMtkBtPatcher->oldHostDeviceRequest)(that, provider, request, data, descriptor, length, completion, timeout);
}

#pragma mark - HCI Read Clock for MediaTek

/*
 * bluetoothd reads the BT clock with HCI_Read_Clock(handle, Which_Clock=0).
 * For the local clock MediaTek returns Connection_Handle 0 instead of the
 * handle that was passed, so bluetoothd cannot match the result to the
 * connection ("Failed to read BT clock"). AAC-ELD streaming to AirPods
 * timestamps audio with that clock, and the headphones never start playing.
 * Ask for the piconet clock of the connection instead (Which_Clock=1): the
 * controller then returns the handle, and as central the piconet clock is
 * our own native clock.
 */
#define HCI_OP_READ_CLOCK 0x1407
static uint32_t readClockRewritten, aclPbRewritten;

static void publishCounters()
{
    IOService *res = IOService::getResourceService();
    if (!res)
        return;
    OSDictionary *d = OSDictionary::withCapacity(2);
    if (!d)
        return;
    OSNumber *n1 = OSNumber::withNumber(readClockRewritten, 32);
    OSNumber *n2 = OSNumber::withNumber(aclPbRewritten, 32);
    if (n1) { d->setObject("ReadClockRewritten", n1); n1->release(); }
    if (n2) { d->setObject("AclPbRewritten", n2); n2->release(); }
    res->setProperty("SimpleMtkBtPatcher", d);
    d->release();
}

void CSimpleMtkBtPatcher::fixReadClock(StandardUSB::DeviceRequest &request, void *data, IOMemoryDescriptor *descriptor)
{
    /* HCI command: class request, host to device */
    if ((request.bmRequestType & 0x80) != 0 || (request.bmRequestType & 0x60) != 0x20 || request.wLength != 6)
        return;
    uint8_t cmd[6];
    if (data != nullptr) {
        memcpy(cmd, data, sizeof(cmd));
    } else if (descriptor != nullptr && descriptor->prepare(kIODirectionOut) == kIOReturnSuccess) {
        /*
         * Read with a read-only prepare and ask for write access only for
         * the command to rewrite: write access on a buffer already
         * prepared read-only fails and makes the kernel log a backtrace.
         */
        bool ok = descriptor->readBytes(0, cmd, sizeof(cmd)) == sizeof(cmd);
        descriptor->complete(kIODirectionOut);
        if (ok && (cmd[0] | (cmd[1] << 8)) == HCI_OP_READ_CLOCK && cmd[2] == 3 && cmd[5] == 0 &&
            descriptor->prepare(kIODirectionInOut) == kIOReturnSuccess) {
            cmd[5] = 1;
            if (descriptor->writeBytes(5, &cmd[5], 1) == 1) {
                readClockRewritten++;
                publishCounters();
            }
            descriptor->complete(kIODirectionInOut);
        }
        return;
    } else {
        return;
    }
    if ((cmd[0] | (cmd[1] << 8)) == HCI_OP_READ_CLOCK && cmd[2] == 3 && cmd[5] == 0) {
        static_cast<uint8_t *>(data)[5] = 1;
        readClockRewritten++;
        publishCounters();
    }
}

#pragma mark - Non-flushable ACL for MediaTek

/*
 * bluetoothd sends A2DP media as "first automatically flushable" ACL
 * fragments (packet boundary flag 0b10). MediaTek firmware flushes those
 * when a large L2CAP frame (AAC-ELD to AirPods, ~966 bytes) does not make
 * it in one baseband packet, so the headphones get broken frames, never
 * start playback and pause the stream. Linux always sends ACL data as
 * non-flushable (0b00) to these controllers. Rewrite the flag on HCI ACL
 * packets going to bulk OUT endpoints.
 */
static uint32_t pbRewritten;

IOReturn CSimpleMtkBtPatcher::newPipeIo(void *that, IOMemoryDescriptor *dataBuffer, uint32_t dataBufferLength, IOUSBHostCompletion *completion, uint32_t completionTimeoutMs)
{
    IOUSBHostPipe *pipe = static_cast<IOUSBHostPipe *>(that);
    if (_mtkAddressValid && dataBuffer != nullptr && pipe->getDeviceAddress() == _mtkAddress) {
        const StandardUSB::EndpointDescriptor *ep = pipe->getEndpointDescriptor();
        const uint8_t type = ep != nullptr ? (ep->bmAttributes & 0x03) : 0;
        /* interrupt IN carries the HCI events, bulk IN the ACL data from the controller */
        if (ep != nullptr && (ep->bEndpointAddress & 0x80) != 0 && completion != nullptr && (type == 0x03 || type == 0x02))
            return sleepWatchIn(that, dataBuffer, dataBufferLength, completion, completionTimeoutMs, type == 0x03);
        /*
         * Only the MediaTek controller's bulk OUT transfers reach this branch.
         * Read the header with a read-only prepare and ask for write access
         * only for an ACL packet to rewrite: write access on a buffer its
         * owner has already prepared read-only fails and makes the kernel log
         * a backtrace.
         */
        if (ep != nullptr &&
            type == 0x02 &&                             /* bulk */
            (ep->bEndpointAddress & 0x80) == 0 &&       /* OUT */
            dataBufferLength >= 4 && dataBufferLength <= 1100 &&
            dataBuffer->prepare(kIODirectionOut) == kIOReturnSuccess) {
            uint8_t hdr[16];   /* ACL header, L2CAP header and the start of a signaling command */
            uint16_t handle = 0, len = 0;
            bool acl = false;
            const uint32_t got = static_cast<uint32_t>(dataBuffer->readBytes(0, hdr, min(dataBufferLength, static_cast<uint32_t>(sizeof(hdr)))));
            if (got >= 4) {
                handle = hdr[0] | (hdr[1] << 8);
                len = hdr[2] | (hdr[3] << 8);
                /* looks like HCI ACL: sane handle, length matches, PB = 0b10 */
                acl = (handle & 0x0fff) < 0x0f00 && len + 4u == dataBufferLength &&
                    ((handle >> 12) & 3) == 2;
            }
            dataBuffer->complete(kIODirectionOut);
            if (got >= 4 && (handle & 0x0fff) < 0x0f00 && len + 4u == dataBufferLength)
                sleepParseAcl(hdr, got, dataBufferLength, false);
            if (acl && dataBuffer->prepare(kIODirectionInOut) == kIOReturnSuccess) {
                hdr[1] &= ~0x30;
                if (dataBuffer->writeBytes(1, &hdr[1], 1) == 1) {
                    if (pbRewritten++ < 3)
                        SYSLOG(DRV_NAME, "ACL PB flushable->non-flushable handle 0x%03x len %u", handle & 0x0fff, len);
                    if ((++aclPbRewritten & 0x3ff) == 1)
                        publishCounters();
                }
                dataBuffer->complete(kIODirectionInOut);
            }
        }
    }
    return FunctionCast(newPipeIo, callbackMtkBtPatcher->oldPipeIo)(that, dataBuffer, dataBufferLength, completion, completionTimeoutMs);
}

#pragma mark - Close classic non-HID links at system sleep

/*
 * With a classic link up (A2DP headphones, even paused) the controller has
 * traffic for the host while the Mac sleeps, signals USB remote wakeup and
 * the Mac is awake again within milliseconds: it never stays asleep. A Mac
 * tells its Broadcom controller which events may wake the host
 * (bluetoothd's "VS event mask for sleep", vendor command OCF 0x103), which
 * fails on MediaTek; that mask only covers Broadcom's vendor events anyway,
 * not the ACL data and standard events of a link.
 *
 * Do what Linux does before suspend (hci_suspend_sync): close the links that
 * must not wake the host. Classic links that carry HID (L2CAP PSM 0x11 or
 * 0x13) and LE links stay, so keyboards and mice keep waking the Mac; every
 * other classic ACL link (A2DP, AVRCP, HFP...) is disconnected with reason
 * 0x15, remote device powered off, as Linux does, before USB suspends.
 *
 * Links are learned from the HCI events on interrupt IN (connection and
 * disconnection complete) and from the L2CAP connection requests on bulk IN
 * and OUT. The last HCI commands with their status, the links with their
 * PSMs and what happened at each sleep and wake are published on
 * IOResources -> SimpleMtkBtSleep. -smbtsleepoff turns all of this off; the
 * sysctl debug.smbtp.sleepdisc=0 only leaves the links alone at run time.
 */

#define HCI_OP_DISCONNECT           0x0406
#define HCI_OP_VS_EVENT_MASK        0xfd03      /* vendor OCF 0x103: bluetoothd's "VS event mask" */
#define HCI_EV_CONN_COMPLETE        0x03
#define HCI_EV_DISCONN_COMPLETE     0x05
#define HCI_EV_CMD_COMPLETE         0x0e
#define HCI_EV_CMD_STATUS           0x0f
#define HCI_EV_SYNC_CONN_COMPLETE   0x2c
#define HCI_EV_LE_META              0x3e
#define HCI_ERR_REMOTE_POWER_OFF    0x15
#define L2CAP_CID_SIGNALING         0x0001
#define L2CAP_CONN_REQ              0x02
#define L2CAP_PSM_HID_CONTROL       0x0011
#define L2CAP_PSM_HID_INTERRUPT     0x0013
#define STATUS_PENDING              0xffff

enum : uint8_t { LinkFree, LinkAcl, LinkSco, LinkLe };

struct SleepLink {
    uint16_t handle;
    uint8_t type;
    bool hid;
    bool early;         /* learned from an L2CAP request read before its connection complete event */
    uint8_t npsm;
    uint16_t psm[6];
};

struct SleepEvent {
    uint32_t sec;
    uint16_t ms;
    char text[58];
};

struct SleepCommand {
    uint32_t sec;
    uint16_t ms;
    uint16_t opcode;
    uint16_t status;
};

struct InWatch {
    IOUSBHostCompletion orig;
    IOMemoryDescriptor *buffer;
    bool events;
};

static IOSimpleLock *sleepLock;
static SleepLink sleepLinks[16];
static SleepEvent sleepEvents[64];
static uint32_t sleepEventNext;
static SleepCommand sleepCommands[48];
static uint32_t sleepCommandNext;
static uint32_t vsMaskSeen, vsMaskStatus = 0xffffffff, sleepCount, disconnectsSent, disconnectsDone;
static uint16_t sleepAwait[16];
static uint32_t sleepAwaitCount;
static uint32_t eventLeft, aclInLeft;          /* bytes still due of an event / ACL packet split over transfers */
static IOUSBHostDevice *cmdDevice;
static IOService *cmdClient;
static StandardUSB::DeviceRequest cmdRequest;
static bool cmdRequestValid;
static thread_call_t publishCall, registerCall;
static IONotifier *sleepNotifier;
static bool sleepRegisterQueued;
static int sleepDisconnect = 1;

SYSCTL_NODE(_debug, OID_AUTO, smbtp, CTLFLAG_RW | CTLFLAG_LOCKED, 0, "SimpleMtkBtPatcher");
SYSCTL_INT(_debug_smbtp, OID_AUTO, sleepdisc, CTLFLAG_RW | CTLFLAG_LOCKED, &sleepDisconnect, 0,
    "1 = close classic non-HID links at system sleep");

static void stamp(uint32_t &sec, uint16_t &ms)
{
    clock_sec_t s = 0;
    clock_usec_t us = 0;
    clock_get_calendar_microtime(&s, &us);
    sec = static_cast<uint32_t>(s);
    ms = static_cast<uint16_t>(us / 1000);
}

static void publishSoon()
{
    if (publishCall != nullptr)
        thread_call_enter(publishCall);
}

static void sleepLog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void sleepLog(const char *fmt, ...)
{
    if (sleepLock == nullptr)
        return;
    char text[sizeof(SleepEvent::text)];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    uint32_t sec;
    uint16_t ms;
    stamp(sec, ms);
    IOSimpleLockLock(sleepLock);
    SleepEvent &e = sleepEvents[sleepEventNext++ % arrsize(sleepEvents)];
    e.sec = sec;
    e.ms = ms;
    strlcpy(e.text, text, sizeof(e.text));
    IOSimpleLockUnlock(sleepLock);
}

static const char *linkTypeName(uint8_t type)
{
    return type == LinkAcl ? "ACL" : type == LinkSco ? "SCO" : type == LinkLe ? "LE" : "?";
}

/* the functions below that touch the tables are called with sleepLock held */

static SleepLink *linkFind(uint16_t handle)
{
    for (auto &l : sleepLinks)
        if (l.type != LinkFree && l.handle == handle)
            return &l;
    return nullptr;
}

static bool linkAdd(uint16_t handle, uint8_t type, bool early = false)
{
    SleepLink *l = linkFind(handle);
    if (l != nullptr && l->early && l->type == type && !early) {
        /* the events and the ACL data come on two pipes: keep what the earlier L2CAP request taught */
        l->early = false;
        return true;
    }
    for (size_t i = 0; l == nullptr && i < arrsize(sleepLinks); i++)
        if (sleepLinks[i].type == LinkFree)
            l = &sleepLinks[i];
    if (l == nullptr)
        return false;
    memset(l, 0, sizeof(*l));
    l->handle = handle;
    l->type = type;
    l->early = early;
    return true;
}

static void commandResult(uint16_t opcode, uint8_t status)
{
    for (uint32_t i = 0; i < arrsize(sleepCommands) && i < sleepCommandNext; i++) {
        SleepCommand &c = sleepCommands[(sleepCommandNext - 1 - i) % arrsize(sleepCommands)];
        if (c.opcode == opcode && c.status == STATUS_PENDING) {
            c.status = status;
            return;
        }
    }
}

static void sleepParseEvent(const uint8_t *b, uint32_t have, uint32_t total)
{
    char line[sizeof(SleepEvent::text)] = {0};
    IOSimpleLockLock(sleepLock);
    if (eventLeft != 0) {
        eventLeft = total >= eventLeft ? 0 : eventLeft - total;
        IOSimpleLockUnlock(sleepLock);
        return;
    }
    if (have < 2) {
        IOSimpleLockUnlock(sleepLock);
        return;
    }
    const uint8_t code = b[0], len = b[1];
    if (2u + len > total)
        eventLeft = 2u + len - total;
    const uint8_t *p = b + 2;
    const uint32_t n = min(have - 2, static_cast<uint32_t>(len));
    switch (code) {
        case HCI_EV_CONN_COMPLETE:
            if (n >= 10 && p[0] == 0) {
                const uint16_t handle = (p[1] | (p[2] << 8)) & 0x0fff;
                const uint8_t type = p[9] == 0x01 ? LinkAcl : LinkSco;
                const bool kept = linkAdd(handle, type);
                snprintf(line, sizeof(line), "connected 0x%03x %s%s", handle, linkTypeName(type), kept ? "" : ", table full");
            }
            break;
        case HCI_EV_SYNC_CONN_COMPLETE:
            if (n >= 3 && p[0] == 0) {
                const uint16_t handle = (p[1] | (p[2] << 8)) & 0x0fff;
                linkAdd(handle, LinkSco);
                snprintf(line, sizeof(line), "connected 0x%03x SCO", handle);
            }
            break;
        case HCI_EV_LE_META:
            if (n >= 4 && (p[0] == 0x01 || p[0] == 0x0a || p[0] == 0x29) && p[1] == 0) {
                const uint16_t handle = (p[2] | (p[3] << 8)) & 0x0fff;
                linkAdd(handle, LinkLe);
                snprintf(line, sizeof(line), "connected 0x%03x LE", handle);
            }
            break;
        case HCI_EV_DISCONN_COMPLETE:
            if (n >= 4 && p[0] == 0) {
                const uint16_t handle = (p[1] | (p[2] << 8)) & 0x0fff;
                SleepLink *l = linkFind(handle);
                bool ours = false;
                for (uint32_t i = 0; i < sleepAwaitCount; i++) {
                    if (sleepAwait[i] == handle) {
                        sleepAwait[i] = sleepAwait[--sleepAwaitCount];
                        disconnectsDone++;
                        ours = true;
                        break;
                    }
                }
                snprintf(line, sizeof(line), "disconnected 0x%03x %s reason 0x%02x%s", handle,
                         l != nullptr ? linkTypeName(l->type) : "?", p[3], ours ? ", ours at sleep" : "");
                if (l != nullptr)
                    l->type = LinkFree;
            }
            break;
        case HCI_EV_CMD_COMPLETE:
            if (n >= 4) {
                const uint16_t opcode = static_cast<uint16_t>(p[1] | (p[2] << 8));
                commandResult(opcode, p[3]);
                if (opcode == HCI_OP_VS_EVENT_MASK) {
                    vsMaskStatus = p[3];
                    snprintf(line, sizeof(line), "command complete 0x%04x status 0x%02x", opcode, p[3]);
                }
            }
            break;
        case HCI_EV_CMD_STATUS:
            if (n >= 4) {
                const uint16_t opcode = static_cast<uint16_t>(p[2] | (p[3] << 8));
                commandResult(opcode, p[0]);
                if (opcode == HCI_OP_VS_EVENT_MASK)
                    vsMaskStatus = p[0];
                if (opcode == HCI_OP_VS_EVENT_MASK || (opcode == HCI_OP_DISCONNECT && sleepAwaitCount != 0))
                    snprintf(line, sizeof(line), "command status 0x%04x status 0x%02x", opcode, p[0]);
            }
            break;
    }
    IOSimpleLockUnlock(sleepLock);
    if (line[0] != 0) {
        sleepLog("%s", line);
        publishSoon();
    }
}

static void sleepParseAcl(const uint8_t *b, uint32_t have, uint32_t total, bool in)
{
    if (sleepLock == nullptr)
        return;
    uint16_t handle = 0, psm = 0;
    bool added = false;
    IOSimpleLockLock(sleepLock);
    if (in && aclInLeft != 0) {
        aclInLeft = total >= aclInLeft ? 0 : aclInLeft - total;
        IOSimpleLockUnlock(sleepLock);
        return;
    }
    if (have >= 4) {
        const uint16_t h = static_cast<uint16_t>(b[0] | (b[1] << 8)), len = static_cast<uint16_t>(b[2] | (b[3] << 8));
        handle = h & 0x0fff;
        if (in && 4u + len > total)
            aclInLeft = 4u + len - total;
        /* first fragment of an L2CAP frame on the classic signaling channel: Connection Request */
        if (((h >> 12) & 3) != 1 && have >= 14 && (b[6] | (b[7] << 8)) == L2CAP_CID_SIGNALING && b[8] == L2CAP_CONN_REQ) {
            psm = static_cast<uint16_t>(b[12] | (b[13] << 8));
            SleepLink *l = linkFind(handle);
            /* signaling channel 1 exists only on classic links: one not seen yet is a classic ACL link */
            if (l == nullptr && linkAdd(handle, LinkAcl, true))
                l = linkFind(handle);
            if (l != nullptr && l->type == LinkAcl) {
                bool known = false;
                for (uint8_t i = 0; i < l->npsm; i++)
                    known = known || l->psm[i] == psm;
                if (!known && l->npsm < arrsize(l->psm)) {
                    l->psm[l->npsm++] = psm;
                    added = true;
                }
                if (psm == L2CAP_PSM_HID_CONTROL || psm == L2CAP_PSM_HID_INTERRUPT)
                    l->hid = true;
            }
        }
    }
    IOSimpleLockUnlock(sleepLock);
    if (added) {
        sleepLog("psm 0x%04x on 0x%03x, from the %s", psm, handle, in ? "device" : "Mac");
        publishSoon();
    }
}

static void sleepInDone(void *owner, void *parameter, IOReturn status, uint32_t bytesTransferred)
{
    InWatch *w = static_cast<InWatch *>(owner);
    const IOUSBHostCompletion orig = w->orig;
    if (status == kIOReturnSuccess && bytesTransferred != 0) {
        uint8_t b[16];
        const uint32_t have = static_cast<uint32_t>(w->buffer->readBytes(0, b, min(bytesTransferred, static_cast<uint32_t>(sizeof(b)))));
        if (w->events)
            sleepParseEvent(b, have, bytesTransferred);
        else
            sleepParseAcl(b, have, bytesTransferred, true);
    }
    w->buffer->complete(kIODirectionIn);
    w->buffer->release();
    IOFree(w, sizeof(*w));
    if (orig.action != nullptr)
        orig.action(orig.owner, orig.parameter, status, bytesTransferred);
}

/*
 * Read what the controller sends to bluetoothd: the transfer goes on with
 * our completion, which reads the first bytes and then calls bluetoothd's.
 * The pipe unwires the buffer before it calls the completion, and reading
 * an unwired descriptor panics (IOGMD: not wired for the IODMACommand), so
 * the buffer is prepared here, where the caller may block, and completed
 * once our completion has read it. A buffer that cannot be prepared goes
 * on unwatched.
 */
static IOReturn sleepWatchIn(void *that, IOMemoryDescriptor *buffer, uint32_t length, IOUSBHostCompletion *completion, uint32_t timeoutMs, bool events)
{
    auto io = FunctionCast(CSimpleMtkBtPatcher::newPipeIo, callbackMtkBtPatcher->oldPipeIo);
    InWatch *w = sleepLock != nullptr ? static_cast<InWatch *>(IOMalloc(sizeof(InWatch))) : nullptr;
    if (w == nullptr)
        return io(that, buffer, length, completion, timeoutMs);
    if (buffer->prepare(kIODirectionIn) != kIOReturnSuccess) {
        IOFree(w, sizeof(*w));
        return io(that, buffer, length, completion, timeoutMs);
    }
    w->orig = *completion;
    w->buffer = buffer;
    w->events = events;
    buffer->retain();
    IOUSBHostCompletion mine { w, sleepInDone, nullptr };
    const IOReturn ret = io(that, buffer, length, &mine, timeoutMs);
    if (ret != kIOReturnSuccess) {
        /* not queued: the completion will not be called */
        buffer->complete(kIODirectionIn);
        buffer->release();
        IOFree(w, sizeof(*w));
    }
    return ret;
}

/*
 * Called from the deviceRequest hook for every HCI command bluetoothd sends:
 * only memory is touched here, no USB I/O. The device, its client and the
 * request are kept so that the sleep handler can send its own commands the
 * same way.
 */
static void sleepNoteCommand(void *device, IOService *client, const StandardUSB::DeviceRequest &request, uint16_t opcode)
{
    if (sleepLock == nullptr)
        return;
    IOUSBHostDevice *dev = OSDynamicCast(IOUSBHostDevice, static_cast<OSObject *>(device));
    IOUSBHostDevice *oldDevice = nullptr;
    IOService *oldClient = nullptr;
    uint32_t sec;
    uint16_t ms;
    stamp(sec, ms);
    IOSimpleLockLock(sleepLock);
    SleepCommand &c = sleepCommands[sleepCommandNext++ % arrsize(sleepCommands)];
    c.sec = sec;
    c.ms = ms;
    c.opcode = opcode;
    c.status = STATUS_PENDING;
    if (opcode == HCI_OP_VS_EVENT_MASK)
        vsMaskSeen++;
    if (dev != nullptr && client != nullptr) {
        if (dev != cmdDevice || client != cmdClient) {
            oldDevice = cmdDevice;
            oldClient = cmdClient;
            dev->retain();
            client->retain();
            cmdDevice = dev;
            cmdClient = client;
        }
        cmdRequest = request;
        cmdRequestValid = true;
    }
    IOSimpleLockUnlock(sleepLock);
    OSSafeReleaseNULL(oldDevice);
    OSSafeReleaseNULL(oldClient);
    if (opcode == HCI_OP_VS_EVENT_MASK) {
        sleepLog("command 0xfd03 (VS event mask) from bluetoothd");
        publishSoon();
    }
}

static IOReturn sleepSendCommand(const uint8_t *cmd, uint16_t len)
{
    IOSimpleLockLock(sleepLock);
    IOUSBHostDevice *dev = cmdDevice;
    IOService *client = cmdClient;
    StandardUSB::DeviceRequest request = cmdRequest;
    const bool valid = cmdRequestValid && dev != nullptr && client != nullptr;
    if (valid) {
        dev->retain();
        client->retain();
    }
    IOSimpleLockUnlock(sleepLock);
    if (!valid)
        return kIOReturnNotReady;
    IOReturn ret = kIOReturnNotAttached;
    if (!dev->isInactive() && !client->isInactive()) {
        ret = kIOReturnNoMemory;
        IOBufferMemoryDescriptor *md = IOBufferMemoryDescriptor::withBytes(cmd, len, kIODirectionOut);
        if (md != nullptr) {
            ret = md->prepare(kIODirectionOut);
            if (ret == kIOReturnSuccess) {
                unsigned int transferred = 0;
                request.wLength = len;
                ret = FunctionCast(CSimpleMtkBtPatcher::newHostDeviceRequest, callbackMtkBtPatcher->oldHostDeviceRequest)(
                    dev, client, request, nullptr, md, transferred, nullptr, 1000);
                md->complete(kIODirectionOut);
            }
            md->release();
        }
    }
    dev->release();
    client->release();
    return ret;
}

static void sleepCloseLinks()
{
    uint16_t targets[arrsize(sleepLinks)];
    uint32_t count = 0, hid = 0, le = 0, sent = 0;
    IOSimpleLockLock(sleepLock);
    sleepCount++;
    for (auto &l : sleepLinks) {
        if (l.type == LinkLe)
            le++;
        else if (l.type == LinkAcl && l.hid)
            hid++;
        else if (l.type == LinkAcl)
            targets[count++] = l.handle;
    }
    const bool enabled = sleepDisconnect != 0;
    sleepAwaitCount = 0;
    IOSimpleLockUnlock(sleepLock);
    sleepLog("sleep: %u classic non-HID, %u HID, %u LE%s", count, hid, le, enabled ? "" : ", closing off");
    if (!enabled || count == 0) {
        publishSoon();
        return;
    }
    for (uint32_t i = 0; i < count; i++) {
        const uint8_t cmd[6] = { HCI_OP_DISCONNECT & 0xff, HCI_OP_DISCONNECT >> 8, 3,
            static_cast<uint8_t>(targets[i] & 0xff), static_cast<uint8_t>(targets[i] >> 8), HCI_ERR_REMOTE_POWER_OFF };
        /* wait for this handle before sending: the completion can come back at once */
        IOSimpleLockLock(sleepLock);
        sleepAwait[sleepAwaitCount++] = targets[i];
        IOSimpleLockUnlock(sleepLock);
        const IOReturn ret = sleepSendCommand(cmd, sizeof(cmd));
        IOSimpleLockLock(sleepLock);
        if (ret == kIOReturnSuccess) {
            disconnectsSent++;
            sent++;
        } else {
            for (uint32_t j = 0; j < sleepAwaitCount; j++)
                if (sleepAwait[j] == targets[i]) {
                    sleepAwait[j] = sleepAwait[--sleepAwaitCount];
                    break;
                }
        }
        IOSimpleLockUnlock(sleepLock);
        sleepLog("disconnect 0x%03x reason 0x15 sent: 0x%x", targets[i], ret);
    }
    uint32_t left = sent;
    for (int t = 0; t < 150 && left != 0; t++) {
        IOSleep(10);
        IOSimpleLockLock(sleepLock);
        left = sleepAwaitCount;
        IOSimpleLockUnlock(sleepLock);
    }
    IOSimpleLockLock(sleepLock);
    sleepAwaitCount = 0;
    IOSimpleLockUnlock(sleepLock);
    if (left != 0)
        sleepLog("sleep: %u of %u disconnections not complete in 1.5 s", left, sent);
    else
        sleepLog("sleep: %u of %u links closed", sent, count);
    publishSoon();
}

static IOReturn sleepWakeHandler(void *target, void *refCon, UInt32 messageType, IOService *provider, void *messageArgument, vm_size_t argSize)
{
    if (messageType == kIOMessageSystemWillSleep) {
        sleepCloseLinks();
    } else if (messageType == kIOMessageSystemHasPoweredOn) {
        sleepLog("wake");
        publishSoon();
    }
    return kIOReturnSuccess;
}

static void sleepRegister(thread_call_param_t, thread_call_param_t)
{
    if (sleepNotifier == nullptr)
        sleepNotifier = registerPrioritySleepWakeInterest(sleepWakeHandler, nullptr, nullptr);
    sleepLog("sleep notification %s", sleepNotifier != nullptr ? "registered" : "not registered");
    publishSoon();
}

/* first sight of the controller: register for sleep from a thread call, never from the hook */
static void sleepNoteDevice()
{
    if (registerCall != nullptr && !__atomic_exchange_n(&sleepRegisterQueued, true, __ATOMIC_RELAXED))
        thread_call_enter(registerCall);
}

static void addString(OSArray *array, const char *s)
{
    OSString *str = OSString::withCString(s);
    if (str != nullptr) {
        array->setObject(str);
        str->release();
    }
}

static void sleepPublish(thread_call_param_t, thread_call_param_t)
{
    IOService *res = IOService::getResourceService();
    if (res == nullptr || sleepLock == nullptr)
        return;
    SleepEvent *events = static_cast<SleepEvent *>(IOMalloc(sizeof(sleepEvents)));
    SleepCommand *commands = static_cast<SleepCommand *>(IOMalloc(sizeof(sleepCommands)));
    SleepLink links[arrsize(sleepLinks)];
    OSDictionary *d = OSDictionary::withCapacity(9);
    OSArray *ev = OSArray::withCapacity(arrsize(sleepEvents));
    OSArray *cm = OSArray::withCapacity(arrsize(sleepCommands));
    OSArray *lk = OSArray::withCapacity(arrsize(sleepLinks));
    if (events != nullptr && commands != nullptr && d != nullptr && ev != nullptr && cm != nullptr && lk != nullptr) {
        IOSimpleLockLock(sleepLock);
        memcpy(events, sleepEvents, sizeof(sleepEvents));
        memcpy(commands, sleepCommands, sizeof(sleepCommands));
        memcpy(links, sleepLinks, sizeof(sleepLinks));
        const uint32_t eventNext = sleepEventNext, commandNext = sleepCommandNext;
        const uint32_t nums[] = { static_cast<uint32_t>(sleepDisconnect), sleepCount, disconnectsSent, disconnectsDone, vsMaskSeen, vsMaskStatus };
        IOSimpleLockUnlock(sleepLock);
        static const char *keys[] = { "Enabled", "Sleeps", "DisconnectsSent", "DisconnectsDone", "VsMaskSeen", "VsMaskStatus" };
        for (size_t i = 0; i < arrsize(keys); i++) {
            OSNumber *num = OSNumber::withNumber(nums[i], 32);
            if (num != nullptr) {
                d->setObject(keys[i], num);
                num->release();
            }
        }
        char s[96];
        for (uint32_t i = eventNext - min(eventNext, static_cast<uint32_t>(arrsize(sleepEvents))); i != eventNext; i++) {
            const SleepEvent &e = events[i % arrsize(sleepEvents)];
            snprintf(s, sizeof(s), "%u.%03u %s", e.sec, e.ms, e.text);
            addString(ev, s);
        }
        for (uint32_t i = commandNext - min(commandNext, static_cast<uint32_t>(arrsize(sleepCommands))); i != commandNext; i++) {
            const SleepCommand &c = commands[i % arrsize(sleepCommands)];
            if (c.status == STATUS_PENDING)
                snprintf(s, sizeof(s), "%u.%03u 0x%04x", c.sec, c.ms, c.opcode);
            else
                snprintf(s, sizeof(s), "%u.%03u 0x%04x -> 0x%02x", c.sec, c.ms, c.opcode, c.status);
            addString(cm, s);
        }
        for (auto &l : links) {
            if (l.type == LinkFree)
                continue;
            int k = snprintf(s, sizeof(s), "0x%03x %s%s%s", l.handle, linkTypeName(l.type), l.hid ? " HID" : "", l.early ? " (no connection event)" : "");
            for (uint8_t i = 0; i < l.npsm && k > 0 && k < static_cast<int>(sizeof(s)); i++)
                k += snprintf(s + k, sizeof(s) - static_cast<size_t>(k), " 0x%04x", l.psm[i]);
            addString(lk, s);
        }
        d->setObject("Links", lk);
        d->setObject("Events", ev);
        d->setObject("Commands", cm);
        res->setProperty("SimpleMtkBtSleep", d);
    }
    OSSafeReleaseNULL(d);
    OSSafeReleaseNULL(ev);
    OSSafeReleaseNULL(cm);
    OSSafeReleaseNULL(lk);
    if (events != nullptr)
        IOFree(events, sizeof(sleepEvents));
    if (commands != nullptr)
        IOFree(commands, sizeof(sleepCommands));
}

static void sleepInit()
{
    /*
     * The way out if the link tracking misbehaves: without the lock and the
     * thread calls no IN transfer is wrapped, no command is noted and nothing
     * registers for sleep, as before this feature.
     */
    if (checkKernelArgument("-smbtsleepoff"))
        return;
    sleepLock = IOSimpleLockAlloc();
    publishCall = thread_call_allocate(sleepPublish, nullptr);
    registerCall = thread_call_allocate(sleepRegister, nullptr);
    sysctl_register_oid(&sysctl__debug_smbtp);
    sysctl_register_oid(&sysctl__debug_smbtp_sleepdisc);
}
