

#!/usr/bin/env python3

from enum import IntEnum

from migen import *
from migen.genlib import fsm
from migen.genlib import fifo
from migen.genlib import cdc
from migen.fhdl.decorators import ModuleTransformer

from litex.soc.integration.doc import AutoDoc, ModuleDoc
from litex.soc.interconnect import stream
from litex.soc.interconnect import csr_eventmanager as ev

from ..endpoint import EndpointType, EndpointResponse
from ..pid import PID, PIDTypes
from ..sm.transfer import UsbTransfer
from .usbwishbonebridge import USBWishboneBridge

from .eptri import TriEndpointInterface


from litex.soc.interconnect.csr import CSRStorage, CSRStatus, CSRField, CSR, AutoCSR


from litex.soc.interconnect.csr_eventmanager import *
from litex.soc.interconnect import stream
 

# Hack the AutoCSR objects to enable access only via Module attributes.
class CSRTransform(ModuleTransformer):
    def __init__(self, parent):
        self.parent = parent

    def transform_instance(self, i):
        # Capture all the available CSRs, then burn the bridge.
        v = i.get_csrs()
        i.get_csrs = None
        
        for c in v:
            # Skip over modules already exposed, should handle potential renaming here.
            #if hasattr(i, c.name):
            #    pass

            # Attach csr as module attribute
            setattr(i, c.name,c)

            if isinstance(c, CSR):
                ...
            else:
                # Clear the finalise function so these aren't altered further.
                # Maybe not required?
                def _null(*kwargs):
                    ...   
                c.finalize = _null

                # Attach these to our modules submodules,
                # needed to ensure the objects are elaborated?
                self.parent.submodules += c

            # create extra bindings to support dev writing
            if isinstance(c, CSRStorage):
                    # .re is used to determine when .storage has been updated.
                    # so we need to create delayed re signal, we'll rename this to re0
                    setattr(c, "re0", c.re)
                    setattr(c.re0, "name", c.name + '_re0')

                    # Our personal .re signal will then update .re0 alongside .storage
                    setattr(c, "re", Signal(name=c.name + '_re'))
                    c.sync += c.re0.eq(c.re)

                    if hasattr(c, "fields"):
                        setattr(c, "dat_w", Record([]))
                        for a in c.fields.fields:
                            s = Signal(a.size,name=f'{c.name}_{a.name}0')

                            c.sync += If(c.re, 
                                c.storage[a.offset:a.offset + a.size].eq(s)
                            )
                            setattr(c.dat_w, a.name, s)

                    else:
                        # if the CSRStorage doesn't have any fields, just provide .dat_w
                        setattr(c, "dat_w", Signal(c.size))
                        c.sync += If(c.re, c.storage.eq(c.dat_w))

            

class CDCUsb(Module, AutoDoc, ModuleDoc, AutoCSR):
    """DummyUSB Self-Enumerating USB Controller

    This implements a device that simply responds to the most common SETUP packets.
    It is intended to be used alongside the Wishbone debug bridge.
    """

    def __init__(self, iobuf, debug=False, vid=0x1209, pid=0x5bf2,
        product="OrangeCrab CDC",
        manufacturer="GsD"):
        

        self.submodules.phy = phy = ClockDomainsRenamer("usb_12")(CDCUsbPHY(iobuf, debug=debug, vid=vid, pid=pid, product=product, manufacturer=manufacturer))

        # create interface for UART
        self._rxtx = CSR(8)
        self._txfull = CSRStatus()
        self._rxempty = CSRStatus()

        self.submodules.ev = EventManager()
        self.ev.tx = EventSourceProcess()
        self.ev.rx = EventSourceProcess()
        self.ev.finalize()

        self._tuning_word = CSRStorage(32, reset=0)
        
        self._configured = CSR()

        self.sink   = stream.Endpoint([("data", 8)])
        self.source = stream.Endpoint([("data", 8)])

        self.rts = Signal()
        self.dtr = Signal()

        self.async_rst = Signal()

        self.specials += cdc.MultiReg(phy.rts, self.rts)
        self.specials += cdc.MultiReg(phy.dtr, self.dtr)

        self.submodules.configure_pulse = cdc.PulseSynchronizer("sys", "usb_12")
        

        self.comb += [
                #phy.source.connect(self.source),
                #self.sink.connect(phy.sink),

                self.source.connect(phy.source),
                phy.sink.connect(self.sink),

                self.async_rst.eq(phy.dtr),


                self.configure_pulse.i.eq(self._configured.re),
                self.phy.configure_set.eq(self.configure_pulse.o),


            ]

        # TX
        tx_fifo = ClockDomainsRenamer({"write":"sys","read":"usb_12"})(stream.AsyncFIFO([("data", 8)], 4, buffered=False))
        #tx_fifo = ResetInserter()(ClockDomainsRenamer({"write":"sys","read":"sys"})(stream.AsyncFIFO([("data", 8)], 4, buffered=False)))
        #tx_fifo = stream.SyncFIFO([("data", 8)], 4, buffered=True)
        self.submodules += tx_fifo

        self.comb += [
            tx_fifo.sink.valid.eq(self._rxtx.re),
            tx_fifo.sink.data.eq(self._rxtx.r),
            self._txfull.status.eq(~tx_fifo.sink.ready),
            tx_fifo.source.connect(self.source),
            # Generate TX IRQ when tx_fifo becomes non-full
            self.ev.tx.trigger.eq(~tx_fifo.sink.ready)
        ]

        # RX
        rx_fifo = ClockDomainsRenamer({"write":"usb_12","read":"sys"})(stream.AsyncFIFO([("data", 8)], 4, buffered=False))
        #rx_fifo = ResetInserter()(ClockDomainsRenamer({"write":"sys","read":"sys"})(stream.AsyncFIFO([("data", 8)], 4, buffered=False)))
        #rx_fifo = stream.SyncFIFO([("data", 8)], 4, buffered=True)
        self.submodules += rx_fifo

        self.comb += [
            self.sink.connect(rx_fifo.sink),
            self._rxempty.status.eq(~rx_fifo.source.valid),
            self._rxtx.w.eq(rx_fifo.source.data),
            rx_fifo.source.ready.eq(self.ev.rx.clear | (False & self._rxtx.we)),
            # Generate RX IRQ when tx_fifo becomes non-empty
            self.ev.rx.trigger.eq(~rx_fifo.source.valid)
        ]


class CDCUsbPHY(Module, AutoDoc, ModuleDoc):
    """DummyUSB Self-Enumerating USB Controller

    This implements a device that simply responds to the most common SETUP packets.
    It is intended to be used alongside the Wishbone debug bridge.
    """

    def __init__(self, iobuf, debug, vid, pid,
        product,
        manufacturer):

        ModuleDoc.__init__(self)

        # Create the eptri USB interface
        usb = TriEndpointInterface(iobuf, debug=debug)
        #usb.finalize()
        self.submodules.eptri = usb = CSRTransform(self)(usb)


        self.sink   = stream.Endpoint([("data", 8)])
        self.source = stream.Endpoint([("data", 8)])


        self.rts = Signal()
        self.dtr = Signal()

        # Ato attach on power up
        self.comb += [
            usb.pullup_out.dat_w.eq(~ResetSignal()),
            usb.pullup_out.re.eq(1), 
        ]

        def make_usbstr(s):
            usbstr = bytearray(2)
            # The first byte is the number of characters in the string.
            # Because strings are utf_16_le, each character is two-bytes.
            # That leaves 126 bytes as the maximum length
            assert(len(s) <= 126)
            usbstr[0] = (len(s)*2)+2
            usbstr[1] = 3
            usbstr.extend(bytes(s, 'utf_16_le'))
            return list(usbstr)

        # Start with 0x8006
        descriptors = {
            # Config descriptor
            # 80 06 00 02
            0x0002: [
                0x09, # bLength 
                0x02, # bDescriptorType 
                62, 0x00, # wTotalLength
                0x02, # bNumInterfaces
                0x01, # bConfigurationValue
                0x00, # iConfiguration
                0x80, # bmAttributes
                0x32, # bMaxPower

                # Interface descriptor
                0x09, # bLength 
                0x04, # bDescriptorType 
                0x00, # bInterfaceNumber
                0x00, # bAlternateSetting
                0x01, # bNumEndpoints
                0x02, # bInterfaceClass (2: Communications Interface)
                0x02, # bInterfaceSubClass (2: Abstract Control Model)
                0x00, # bInterfacePrototcol
                0x00, # iInterface

                # Header Functional Descriptor
                0x05, # bFunctionLength
                0x24, # bDescriptorType (24: CS_INTERFACE)
                0x00, # bDescriptorSubtype
                0x10, 0x01, # bcdCDC

                # ACM Functional Descriptor
                0x04, # bFunctionLength
                0x24, # bDescriptorType
                0x02, # bDescriptorSubtype
                0x02, # bmCapabilities
                
                # Union Functional Description
                0x05, # bLength
                0x24, # bDescriptorType
                0x06, # bDescriptorSubtype
                0x00, # bControlInterfoce
                0x01, # bSubordinateInterface0
                
                # Control Endpoint Descriptior
                0x07, # bLength
                0x05, # bDescriptorType (5: ENDPOINT)
                0x81, # bEndpointAddress 
                0x03, # bmAttributes (XFER_INTERRUPT)
                0x08, 0x00, # wMaxPacketSize
                0x40, # bInterval

                0x09, # bLength            = sizeof(tusb_desc_interface_t),
                0x04, # bDescriptorType    = TUSB_DESC_TYPE_INTERFACE,
                0x01, # bInterfaceNumber   = 5,
                0x00, # bAlternateSetting  = 0x00,
                0x02, # bNumEndpoints      = 2,
                0x0A, # bInterfaceClass    = TUSB_CLASS_CDC_DATA,
                0x00, # bInterfaceSubClass = 0,
                0x00, # bInterfaceProtocol = 0,
                0x00, # iInterface         = 0x00
            
                0x07, # bLength          = sizeof(tusb_desc_endpoint_t),
                0x05, # bDescriptorType  = TUSB_DESC_TYPE_ENDPOINT,
                0x02, # bEndpointAddress = 5,
                0x02, # bmAttributes     = { .xfer = TUSB_XFER_BULK },
                0x40, 0x00, # wMaxPacketSize   = 64,
                0x00, # bInterval        = 0
            
                0x07, # bLength          = sizeof(tusb_desc_endpoint_t),
                0x05, # bDescriptorType  = TUSB_DESC_TYPE_ENDPOINT,
                0x82, # bEndpointAddress = 0x85,
                0x02, # bmAttributes     = { .xfer = TUSB_XFER_BULK },
                0x40, 0x00, # wMaxPacketSize   = 64,
                0x00, # bInterval        = 0
            ],

            # Device descriptor
            # 80 06 00 01
            0x0001: [
                0x12, # Length
                0x01, # bDescriptorType 
                0x00, 0x02, # bcdUSB
                0x02, # bDeviceClass
                0x00, # bDeviceSubClass
                0x00, # bDeviceProtocol
                0x40, # bMaxPacketSize0
                (vid>>0)&0xff, (vid>>8)&0xff, # idVendor
                (pid>>0)&0xff, (pid>>8)&0xff, # idProduct
                0x01, 0x01, # bcdDevice
                0x01, # iManufacture
                0x02, # iProduct
                0x00, # iSerialNumber
                0x01, # bNumConfigurations
            ],

            # String 0
            0x0003: [
                0x04, 0x03, 0x09, 0x04,
            ],

            # String 1 (manufacturer)
            0x0103: make_usbstr(manufacturer),

            # String 2 (Product)
            0x0203: make_usbstr(product),

            # BOS descriptor
            0x000f: [
                0x05, 0x0f, 0x1d, 0x00, 0x01, 0x18, 0x10, 0x05,
                0x00, 0x38, 0xb6, 0x08, 0x34, 0xa9, 0x09, 0xa0,
                0x47, 0x8b, 0xfd, 0xa0, 0x76, 0x88, 0x15, 0xb6,
                0x65, 0x00, 0x01, 0x02, 0x01,
            ],

            0xee03: [
                0x12, 0x03, 0x4d, 0x53, 0x46, 0x54, 0x31, 0x30,
                0x30, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00,
            ],
        }

        # Starts with 0xc07e or 0xc17e
        usb_ms_compat_id_descriptor = [
            0x28, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00,
            0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x01, 0x57, 0x49, 0x4e, 0x55, 0x53, 0x42,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ]

        class MemoryContents:
            def __init__(self):
                self.contents = [0x00]
                self.offsets = {}
                self.lengths = {}

            def add(self, wRequestAndType, wValue, mem):
                self.offsets[wRequestAndType << 16 | wValue] = len(self.contents)
                self.lengths[wRequestAndType << 16 | wValue] = len(mem)
                self.contents = self.contents + mem

        mem = MemoryContents()
        for key, value in descriptors.items():
            mem.add(0x8006, key, value)

        mem.add(0xc07e, 0x0000, usb_ms_compat_id_descriptor)
        mem.add(0x8000, 0x0000, [0, 0]) # Get device status
        mem.add(0x0009, 0x0100, []) # Set configuration 1

        mem.add(0xA121, 0x0000, [0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08]) # Get line_coding
        #mem.add(0xA120, 0x0000, [0x00,0x00]) # SerialState

        out_buffer = self.specials.out_buffer = Memory(8, len(mem.contents), init=mem.contents)
        self.specials.out_buffer_rd = out_buffer_rd = out_buffer.get_port(write_capable=False, clock_domain="usb_12")
        self.autocsr_exclude = ['out_buffer']

        # Needs to be able to index Memory
        response_addr = Signal(9)
        response_len = Signal(6)
        response_ack = Signal()
        bytes_remaining = Signal(6)
        bytes_addr = Signal(9)

        new_address = Signal(7)

        configured = Signal()
        configured_delay = Signal(16, reset=2**16-1)

        self.configure_set = Signal()

        self.sync += [
            If(self.dtr & (configured_delay > 0), 
                configured_delay.eq(configured_delay - 1)
            ).Elif(self.configure_set,
                configured_delay.eq(0)
            )
        ]
        self.comb += configured.eq(configured_delay == 0)


        # SETUP packets contain a DATA segment that is always 8 bytes.
        # However, we're only ever interested in the first 4 bytes, plus
        # the last byte.
        usbPacket = Signal(32)
        wRequestAndType = Signal(16)
        wValue = Signal(16)
        wLength = Signal(8)
        self.comb += [
            wRequestAndType.eq(usbPacket[16:32]),
            wValue.eq(usbPacket[0:16]),
        ]
        setup_index = Signal(4)

        # Respond to various descriptor requests
        cases = {}
        for key in mem.offsets:
            cases[key] = [
                response_len.eq(mem.lengths[key]),
                response_addr.eq(mem.offsets[key]),
            ]
        self.comb += Case(usbPacket, cases)

        self.submodules.config = config = FSM(reset_state="IDLE")

        toggle = Signal()

        config.act("IDLE",
            #usb.address.dat_w.eq(new_address),
            usb.address.dat_w.addr.eq(new_address),
            usb.address.re.eq(1),
            
            usb.out_ctrl.dat_w.epno.eq(2),
            usb.out_ctrl.dat_w.enable.eq(1),
            usb.out_ctrl.re.eq(1),

            
            NextState("WAIT"),
        )

        config.act("WAIT",
            #usb.in_ctrl.dat_w.epno.eq(0),
            #usb.in_ctrl.re.eq(1),

            usb.out_ctrl.dat_w.epno.eq(0),
            usb.out_ctrl.dat_w.enable.eq(1),
            usb.out_ctrl.re.eq(1),


            If(usb.setup_status.fields.have,
                NextState("SETUP"),
                NextValue(setup_index, 0),
                usb.out_ev_pending.r.eq(1),
                usb.out_ev_pending.re.eq(1),
            
            ).Elif(usb.out_status.fields.have,
                
            ),


            # Data RX?
            If(usb.out_ev_pending.w,
                usb.out_ev_pending.r.eq(1),
                usb.out_ev_pending.re.eq(1),

                


                If((usb.out_status.fields.epno == 2) & usb.out_status.fields.pend,
                    NextState("DRAIN-RX")
                )
            ),



            # UART_FIFO data?
            If(self.source.valid & configured,
                NextState("FILL-TX"),
            )
        )

        delayed_re = Signal()
        config.act("FILL-TX",
            usb.in_data.dat_w.data.eq(self.source.data),

            usb.in_data.re.eq(delayed_re & self.source.valid),
            NextValue(delayed_re,0),

            self.source.ready.eq(delayed_re & self.source.valid),

            If(self.source.valid,
                NextValue(delayed_re,self.source.valid),
            ).Else(
                usb.in_ctrl.dat_w.epno.eq(2),
                usb.in_ctrl.re.eq(1),
                NextState("WAIT-TRANSACTION"),
            )
        )

        # OUT data always captures 2 extra bytes from CRC
        # Since we don't know in advance how long the 
        # transaction was we need to account for this now
        data_d1 = Signal(8)
        re_d1 = Signal()

        data_d2 = Signal(8)
        re_d2 = Signal()

        config.act("DRAIN-RX",
            self.sink.data.eq(data_d2),

            usb.out_data.we.eq(delayed_re & usb.out_status.fields.have & self.sink.ready),
            NextValue(delayed_re,0),

            self.sink.valid.eq(re_d2 & usb.out_status.fields.have & self.sink.ready),

            If(usb.out_status.fields.have,

                NextValue(delayed_re,usb.out_status.fields.have),

                If(self.sink.ready,
                    NextValue(data_d1, usb.out_data.fields.data),
                    NextValue(data_d2, data_d1),

                    NextValue(re_d1, delayed_re),
                    NextValue(re_d2, re_d1),

                )

            ).Else(
                    NextValue(data_d1, 0),
                    NextValue(data_d2, 0),

                    NextValue(re_d1, 0),
                    NextValue(re_d2, 0),
                    NextValue(delayed_re,0),

                    NextState("IDLE"),
            )
        )

        

        config.act("SETUP",
           # read out setup packets to determine what to do 
           If(usb.setup_status.fields.have,
                NextValue(setup_index,setup_index + 1),
                Case(setup_index, {
                    0: NextValue(usbPacket,Cat(usb.setup_data.fields.data, usbPacket[0:24])),
                    1: NextValue(usbPacket,Cat(usb.setup_data.fields.data, usbPacket[0:24])),
                    2: NextValue(usbPacket,Cat(usb.setup_data.fields.data, usbPacket[0:24])),
                    3: NextValue(usbPacket,Cat(usb.setup_data.fields.data, usbPacket[0:24])),
                    # 4: wIndex.eq(data_recv_payload_delayed),
                    # 5: wIndex.eq(Cat(wIndex[0:8], data_recv_payload_delayed)),
                    6: NextValue(wLength,usb.setup_data.fields.data),
                    # 7: wLength.eq(Cat(wLength[0:8], data_recv_payload_delayed)),
                }),
                usb.setup_data.we.eq(1)
            ),
            
            # Determine which state next 
            If(setup_index == 0xA,
                # Ack with a blank IN packet
                usb.in_ctrl.dat_w.epno.eq(0),
                usb.in_ctrl.re.eq(1),
                
                NextState("WAIT-TRANSACTION"),
                If(wRequestAndType == 0x0005,
                    # Set Address
                    NextValue(new_address,wValue[8:15]),
                    NextState("WAIT-TRANSACTION"),
                ).Elif(wRequestAndType == 0x2122,
                    # Set Address
                    NextValue(self.rts,wValue[9]),
                    NextValue(self.dtr,wValue[8]),
                    NextState("WAIT-TRANSACTION"),
                ).Elif((usb.setup_status.fields.is_in) & (response_len > 0),
                    NextState("SETUP-IN"),
                    If(response_len > wLength,
                        NextValue(bytes_remaining,wLength),
                    ).Else(
                        NextValue(bytes_remaining,response_len),
                    ),
                    NextValue(bytes_addr,response_addr),
                ),
            )
        )

        config.act("SETUP-IN",
            usb.in_data.dat_w.data.eq(out_buffer_rd.dat_r),

            usb.in_data.re.eq(delayed_re),
            NextValue(delayed_re,0),

            If(bytes_remaining,
                NextValue(delayed_re,1),
                NextValue(bytes_addr, bytes_addr + 1),
                NextValue(bytes_remaining, bytes_remaining - 1),
            ).Elif(usb.in_ev_pending.w,
                NextState("WAIT-TRANSACTION"),
            )
        ),

        config.act("WAIT-TRANSACTION",

            usb.out_data.we.eq(1),
            If(usb.in_ev_pending.w,
                usb.in_ev_pending.r.eq(1),
                usb.in_ev_pending.re.eq(1),
                
                
                NextState("IDLE"),
            )
        )

        config.act("WAIT-OUT",
            If(usb.in_ev_pending.w & usb.out_ev_pending.w,
                usb.in_ev_pending.r.eq(1),
                usb.in_ev_pending.re.eq(1),

                usb.out_ev_pending.r.eq(1),
                usb.out_ev_pending.re.eq(1),
                
                
                NextState("IDLE"),
            )
        )

        


        self.comb += [
            out_buffer_rd.adr.eq(bytes_addr),
        ]

