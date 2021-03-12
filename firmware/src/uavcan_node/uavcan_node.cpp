/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Author: Pavel Kirienko <pavel.kirienko@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "uavcan_node.hpp"
#include <algorithm>
#include <ch.hpp>
#include <board/board.hpp>
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/config/config.h>	// TODO: remove dependency on the implementation details
#include <unistd.h>
#include <motor/motor.h>
#include <uavcan/node/Heartbeat_1_0.h>
#include <bxcan.h>
#include <canard.h>
#include <canard_dsdl.h>
#include <board/canEnInit.hpp>

namespace uavcan_node
{
namespace
{

os::config::Param<unsigned> param_node_id("uavcan_node_id",   0,      0,       125);

// UAVCAN spin loop
class : public chibios_rt::BaseStaticThread<4000>
{
    uavcan_node_Heartbeat_1_0 node_heartbeat{};
	os::watchdog::Timer wdt_;
	volatile bool need_to_print_status_ = false;
    CanardTransferID message_transfer_id = 0;

    static void* canardAllocate(CanardInstance* const ins, const size_t amount)
    {
        (void) ins;
        return malloc(amount);
    }

    static void canardFree(CanardInstance* const ins, void* const pointer)
    {
        (void) ins;
        free(pointer);
    }

    void handle_background_tasks()
	{
		if (need_to_print_status_) {
			need_to_print_status_ = false;

			//std::printf("CAN bitrate: %u\n", unsigned(active_can_bus_bit_rate));
			//std::printf("Node ID:     %u\n", get_node().getNodeID().get());
            //std::printf("Node mode:   %u\n", node_status_mode);
            //std::printf("Node health: %u\n", node_status_health);

            //const auto perf = get_node().getDispatcher().getTransferPerfCounter();

            //const auto pool_capacity = get_node().getAllocator().getBlockCapacity();
            //const auto pool_peak_usage = get_node().getAllocator().getPeakNumUsedBlocks();

            //uavcan::CanIfacePerfCounters iface_perf[uavcan::MaxCanIfaces];
			//std::uint8_t num_ifaces = 0;
			/*for (num_ifaces = 0;
			     num_ifaces < get_node().getDispatcher().getCanIOManager().getNumIfaces();
			     num_ifaces++)
			{
				iface_perf[num_ifaces] =
					get_node().getDispatcher().getCanIOManager().getIfacePerfCounters(num_ifaces);
			}*/

			//std::printf("Memory pool capacity:   %u blocks\n", pool_capacity);
			//std::printf("Memory pool peak usage: %u blocks\n", pool_peak_usage);

			/*std::printf("Transfers RX/TX: %u / %u\n",
				    unsigned(perf.getRxTransferCount()),
				    unsigned(perf.getTxTransferCount()));
			std::printf("Transfer errors: %u\n", unsigned(perf.getErrorCount()));*/

			/*for (unsigned i = 0; i < num_ifaces; i++)
			{
				std::printf("CAN iface %u:\n", i);
				std::printf("    Frames RX/TX: %u / %u\n",
					    unsigned(iface_perf[i].frames_rx), unsigned(iface_perf[i].frames_tx));
				std::printf("    RX overflows: %u\n",
					    unsigned(can.driver.getIface(i)->getRxQueueOverflowCount()));
				std::printf("    Errors:       %u\n", unsigned(iface_perf[i].errors));
			}*/
		}
	}

	void init_can() {
        wdt_.reset();
        ::sleep(1);
        handle_background_tasks();

        BxCANTimings timings;
        const uint32_t clck_frequency = STM32_PCLK1;
        const uint32_t bitrate = 1000000;
        bool res = bxCANComputeTimings(clck_frequency, bitrate, &timings);
        if (res) {
            ::os::lowsyslog("CAN timings computed at %u bps\n", unsigned(bitrate));
        } else {
            ::os::lowsyslog("Could not compute CAN timing; status: %d, bitrate: %u\n", res, unsigned(bitrate));
        }
        uavcan_stm32::CanEn();
        res = bxCANConfigure(0, timings, false);
        if (res) {
            ::os::lowsyslog("CAN inited at %u bps\n", unsigned(bitrate));
        } else {
            ::os::lowsyslog("Could not init CAN; status: %d,, bitrate: %u\n", res, unsigned(bitrate));
        }
    }

	void init_node()
	{
		/*get_node().setName(NODE_NAME);

		 //Software version
		get_node().setSoftwareVersion(get_uavcan_software_version());

		 //Hardware version
		const auto hw_major_minor = board::detect_hardware_version();

		uavcan::protocol::HardwareVersion hwver;

		hwver.major = hw_major_minor.major;
		hwver.minor = hw_major_minor.minor;

		const auto uid = board::read_unique_id();
		std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));

		{
			board::DeviceSignature signature;
			if (board::try_read_device_signature(signature)) {
				std::copy(std::begin(signature), std::end(signature),
					std::back_inserter(hwver.certificate_of_authenticity));
			}
		}

		get_node().setHardwareVersion(hwver);

	    //Starting the node
		while (true) {
			const int uavcan_start_res = get_node().start();
			if (uavcan_start_res >= 0) {
				break;
			}
			os::lowsyslog("UAVCAN: Node init failure: %i, will retry\n", uavcan_start_res);
			::sleep(1);
			handle_background_tasks();
		}
		assert(get_node().isStarted());


		//Configuring node ID
	        if (param_node_id.get() > 0 || get_inherited_node_id().isUnicast())
	        {
	            get_node().setNodeID((param_node_id.get() > 0) ?
	        		    static_cast<std::uint8_t>(param_node_id.get()) : get_inherited_node_id());

	            os::lowsyslog("UAVCAN: Using static node ID %d\n", int(get_node().getNodeID().get()));
	        }
	        else
	        {
	            uavcan::DynamicNodeIDClient dnid_client(get_node());

	            {
	                const int res = dnid_client.start(
	                	get_node().getNodeStatusProvider().getHardwareVersion().unique_id);
	                if (res < 0)
	                {
				board::die(res);
	                }
	            }

	            os::lowsyslog("UAVCAN: Waiting for dynamic node ID allocation...\n");

	            while (!dnid_client.isAllocationComplete())
	            {
	                get_node().spin(uavcan::MonotonicDuration::fromMSec(100));
			wdt_.reset();
			handle_background_tasks();
	            }

	            os::lowsyslog("UAVCAN: Dynamic node ID %d allocated by %d\n",
	                      int(dnid_client.getAllocatedNodeID().get()), int(dnid_client.getAllocatorNodeID().get()));

	            get_node().setNodeID(dnid_client.getAllocatedNodeID());
	        }

        //Initializing the logic
		get_node().setRestartRequestHandler(&restart_request_handler);

		int res = get_param_server().start(&param_manager);
		if (res < 0) {
			board::die(res);
		}

		res = init_esc_controller(get_node());
		if (res < 0) {
			board::die(res);
		}

		res = init_indication_controller(get_node());
		if (res < 0) {
			board::die(res);
		}

	        res = get_begin_firmware_update_server().start(&handle_begin_firmware_update_request);
	        if (res < 0)
	        {
			board::die(res);
	        }

		enumeration_handler_.construct<uavcan::INode&>(get_node());
		res = enumeration_handler_->start();
		if (res < 0) {
			board::die(res);
		}

		os::lowsyslog("UAVCAN: Node started, ID %i\n", int(get_node().getNodeID().get()));*/
        uavcan_node_Heartbeat_1_0_initialize_(&node_heartbeat);
        node_heartbeat.uptime = 0;
	}

    void publishHeartbeat(CanardInstance* const canard){
        std::size_t msg_size = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
        std::uint8_t buffer[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
        uavcan_node_Heartbeat_1_0_serialize_(&node_heartbeat, buffer,  &msg_size);
        const CanardTransfer transfer = {
                .timestamp_usec = 0,
                .priority       = CanardPriorityNominal,
                .transfer_kind  = CanardTransferKindMessage,
                .port_id        = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
                .remote_node_id = CANARD_NODE_ID_UNSET,
                .transfer_id    = message_transfer_id,
                .payload_size   = msg_size,
                .payload        = buffer
        };
        ++message_transfer_id;
        (void) canardTxPush(canard, &transfer);
    }

public:
	void main() override
	{
		wdt_.startMSec(10000);
		setName("uavcan");

        init_can();

        wdt_.reset();

        init_node();

        wdt_.reset();
        CanardInstance canard = canardInit(&canardAllocate, &canardFree);

        while (!os::isRebootRequested()) {
			wdt_.reset();

			handle_background_tasks();

			//get_node().getNodeStatusProvider().setHealth(node_status_health);
			//get_node().getNodeStatusProvider().setMode(node_status_mode);

			/*const int spin_res = get_node().spin(uavcan::MonotonicDuration::fromMSec(100));
			if (spin_res < 0) {
				os::lowsyslog("UAVCAN: Spin failure: %d\n", spin_res);
			}*/
            /*uavcan_node_Heartbeat_1_0 node_heartbeat = {
                    .uptime = test_uptimeSec,
                    .health = { uavcan_node_Health_1_0_NOMINAL },
                    .mode =  { uavcan_node_Mode_1_0_OPERATIONAL }
            };*/
            node_heartbeat.uptime++;
            ::usleep(1000 * 1000);
            publishHeartbeat(&canard);
        }

		os::lowsyslog("UAVCAN: Going down\n");
		//(void)get_node().spin(uavcan::MonotonicDuration::fromMSec(10));
	}

	void print_status()
	{
		need_to_print_status_ = true;
		while (need_to_print_status_) {
			::usleep(10000);
		}
	}
} node_thread;

}

void print_status()
{
	node_thread.print_status();
}

int init()
{
	(void)node_thread.start(HIGHPRIO - 2);

	return 0;
}

}
