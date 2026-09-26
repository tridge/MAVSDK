#include "mavsdk.hpp"
#include "plugins/action/action.hpp"
#include "plugins/param/param.hpp"
#include "plugins/param_server/param_server.hpp"
#include "plugins/ftp/ftp.hpp"
#include "plugins/mission/mission.hpp"
#include "plugins/mission_raw_server/mission_raw_server.hpp"
#include "plugins/ftp_server/ftp_server.hpp"
#include "fs_helpers.hpp"
#include "plugins/action_server/action_server.hpp"
#include "plugins/mavlink_direct/mavlink_direct.hpp"
#include "plugins/mavlink_direct_server/mavlink_direct_server.hpp"
#include <atomic>
#include <chrono>
#include <future>
#include <thread>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using namespace mavsdk;

// 32 bit system IDs, as proposed in ArduPilot/pymavlink#1229. A system ID
// above 255 no longer fits the MAVLink 2 header's single sysid byte, so the
// sender sets MAVLINK_IFLAG_SYSID32 and the header grows by 3 bytes. A target
// above 255 likewise moves into an extended header behind
// MAVLINK_IFLAG_TARGET32.

static_assert(MAVLINK_TARGET32_HEADER_EXTRA == 4, "Requires the current TARGET32 header");
static_assert(MAVLINK_TARGET_SYSTEM_SENTINEL == 255, "Requires non-broadcast target sentinel");

// 0x0A000001 is 10.0.0.1, which is the point of the feature: an IPv4 address
// used directly as a system ID.
static constexpr uint32_t autopilot_sysid = 0x0A000001;
static constexpr uint32_t groundstation_sysid = 0x0A000002;

TEST(Sysid32, Discovery)
{
    Mavsdk mavsdk_groundstation{
        Mavsdk::Configuration{groundstation_sysid, MAV_COMP_ID_MISSIONPLANNER, false}};
    Mavsdk mavsdk_autopilot{Mavsdk::Configuration{autopilot_sysid, MAV_COMP_ID_AUTOPILOT1, true}};

    ASSERT_EQ(
        mavsdk_groundstation.add_any_connection("udpin://0.0.0.0:17010"),
        ConnectionResult::Success);
    ASSERT_EQ(
        mavsdk_autopilot.add_any_connection("udpout://127.0.0.1:17010"), ConnectionResult::Success);

    auto maybe_system = mavsdk_groundstation.first_autopilot(10.0);
    ASSERT_TRUE(maybe_system);

    // The full 32 bit value has to survive discovery. Truncating would report
    // system 1 here, which is a different (and very common) system.
    EXPECT_EQ(maybe_system.value()->get_system_id(), autopilot_sysid);
}

TEST(Sysid32, CommandRoundtrip)
{
    Mavsdk mavsdk_groundstation{
        Mavsdk::Configuration{groundstation_sysid, MAV_COMP_ID_MISSIONPLANNER, false}};
    Mavsdk mavsdk_autopilot{Mavsdk::Configuration{autopilot_sysid, MAV_COMP_ID_AUTOPILOT1, true}};

    ASSERT_EQ(
        mavsdk_groundstation.add_any_connection("udpin://0.0.0.0:17011"),
        ConnectionResult::Success);
    ASSERT_EQ(
        mavsdk_autopilot.add_any_connection("udpout://127.0.0.1:17011"), ConnectionResult::Success);

    auto action_server = ActionServer{mavsdk_autopilot.server_component()};
    action_server.set_armable(true, true);
    action_server.set_disarmable(true, true);

    auto maybe_system = mavsdk_groundstation.first_autopilot(10.0);
    ASSERT_TRUE(maybe_system);
    auto action = Action{maybe_system.value()};

    // A command is targeted, so this only works if the target system ID makes
    // it into the extended header and the ack finds its way back. Both sides
    // route on the 32 bit value.
    EXPECT_EQ(action.arm(), Action::Result::Success);
    EXPECT_EQ(action.disarm(), Action::Result::Success);
}

TEST(Sysid32, MavlinkDirectRoundtrip)
{
    Mavsdk mavsdk_groundstation{
        Mavsdk::Configuration{groundstation_sysid, MAV_COMP_ID_MISSIONPLANNER, false}};
    Mavsdk mavsdk_autopilot{Mavsdk::Configuration{autopilot_sysid, MAV_COMP_ID_AUTOPILOT1, true}};

    ASSERT_EQ(
        mavsdk_groundstation.add_any_connection("udpin://0.0.0.0:17012"),
        ConnectionResult::Success);
    ASSERT_EQ(
        mavsdk_autopilot.add_any_connection("udpout://127.0.0.1:17012"), ConnectionResult::Success);

    auto maybe_system = mavsdk_groundstation.first_autopilot(10.0);
    ASSERT_TRUE(maybe_system);

    auto receiver = MavlinkDirect{maybe_system.value()};
    auto sender = MavlinkDirectServer{mavsdk_autopilot.server_component()};

    auto prom = std::promise<MavlinkDirect::MavlinkMessage>();
    auto fut = prom.get_future();

    auto handle = receiver.subscribe_message(
        "GLOBAL_POSITION_INT",
        [&prom](MavlinkDirect::MavlinkMessage message) { prom.set_value(message); });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    MavlinkDirectServer::MavlinkMessage message;
    message.message_name = "GLOBAL_POSITION_INT";
    message.fields_json = R"({"time_boot_ms":12345,"lat":473977418,"lon":-1223974560,"alt":100500,)"
                          R"("relative_alt":50250,"vx":100,"vy":-50,"vz":25,"hdg":18000})";

    ASSERT_EQ(sender.send_message(message), MavlinkDirectServer::Result::Success);

    ASSERT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    auto received = fut.get();

    EXPECT_EQ(received.system_id, autopilot_sysid);
    EXPECT_EQ(nlohmann::json::parse(received.fields_json)["lat"], 473977418);

    receiver.unsubscribe_message(handle);
}

TEST(Sysid32, TargetAboveEightBits)
{
    Mavsdk mavsdk_groundstation{
        Mavsdk::Configuration{groundstation_sysid, MAV_COMP_ID_MISSIONPLANNER, false}};
    Mavsdk mavsdk_autopilot{Mavsdk::Configuration{autopilot_sysid, MAV_COMP_ID_AUTOPILOT1, true}};

    ASSERT_EQ(
        mavsdk_groundstation.add_any_connection("udpin://0.0.0.0:17013"),
        ConnectionResult::Success);
    ASSERT_EQ(
        mavsdk_autopilot.add_any_connection("udpout://127.0.0.1:17013"), ConnectionResult::Success);

    auto maybe_system = mavsdk_groundstation.first_autopilot(10.0);
    ASSERT_TRUE(maybe_system);

    auto receiver = MavlinkDirectServer{mavsdk_autopilot.server_component()};
    auto sender = MavlinkDirect{maybe_system.value()};

    auto prom = std::promise<MavlinkDirectServer::MavlinkMessage>();
    auto fut = prom.get_future();

    auto handle = receiver.subscribe_message(
        "COMMAND_LONG",
        [&prom](MavlinkDirectServer::MavlinkMessage message) { prom.set_value(message); });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // The payload's target_system field is only 8 bits wide, so this target
    // has to travel in the extended header instead. Were it truncated it would
    // arrive as 1, and were it zeroed it would look like a broadcast.
    MavlinkDirect::MavlinkMessage message;
    message.message_name = "COMMAND_LONG";
    message.target_system_id = autopilot_sysid;
    message.target_component_id = MAV_COMP_ID_AUTOPILOT1;
    message.fields_json =
        R"({"command":400,"confirmation":0,"param1":1.0,"param2":0.0,"param3":0.0,)"
        R"("param4":0.0,"param5":0.0,"param6":0.0,"param7":0.0})";

    ASSERT_EQ(sender.send_message(message), MavlinkDirect::Result::Success);

    ASSERT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    auto received = fut.get();

    EXPECT_EQ(received.system_id, groundstation_sysid);
    EXPECT_EQ(received.target_system_id, autopilot_sysid);
    EXPECT_EQ(received.target_component_id, MAV_COMP_ID_AUTOPILOT1);

    receiver.unsubscribe_message(handle);
}

TEST(Sysid32, DirectSendUsesOnlyRealTargetFields)
{
    Mavsdk ground{Mavsdk::Configuration{42, MAV_COMP_ID_MISSIONPLANNER, false}};
    Mavsdk vehicle{Mavsdk::Configuration{0xffffffffU, MAV_COMP_ID_AUTOPILOT1, true}};
    ASSERT_EQ(ground.add_any_connection("udpin://0.0.0.0:17015"), ConnectionResult::Success);
    ASSERT_EQ(vehicle.add_any_connection("udpout://127.0.0.1:17015"), ConnectionResult::Success);
    auto system = ground.first_autopilot(10.0);
    ASSERT_TRUE(system);
    MavlinkDirect client{*system};
    MavlinkDirectServer server{vehicle.server_component()};

    for (bool from_server : {false, true}) {
        auto& sender = from_server ? vehicle : ground;
        for (const char* name : {"COMMAND_LONG", "MANUAL_CONTROL", "GLOBAL_POSITION_INT"}) {
            for (uint32_t target : {0U, 7U, 255U, 256U, 0xffffffffU}) {
                const bool has_target = std::string(name) != "GLOBAL_POSITION_INT";
                const uint32_t msgid = std::string(name) == "COMMAND_LONG"   ? 76 :
                                       std::string(name) == "MANUAL_CONTROL" ? 69 :
                                                                               33;
                auto promise = std::make_shared<std::promise<mavlink_message_t>>();
                auto once = std::make_shared<std::atomic<bool>>(false);
                auto future = promise->get_future();
                sender.intercept_outgoing_messages_async([=](mavlink_message_t& frame) {
                    if (frame.msgid == msgid &&
                        (msgid != 76 || mavlink_msg_command_long_get_command(&frame) == 300) &&
                        !once->exchange(true)) {
                        promise->set_value(frame);
                        return false;
                    }
                    return true;
                });
                // For small targets an unspecified API component must preserve
                // the component explicitly supplied in the JSON payload.
                const std::string fields = std::string(name) == "COMMAND_LONG" ?
                                               R"({"command":300,"target_component":1})" :
                                               "{}";
                if (from_server) {
                    MavlinkDirectServer::MavlinkMessage message{};
                    message.message_name = name;
                    message.fields_json = fields;
                    message.target_system_id = target;
                    ASSERT_EQ(server.send_message(message), MavlinkDirectServer::Result::Success);
                } else {
                    MavlinkDirect::MavlinkMessage message{};
                    message.message_name = name;
                    message.fields_json = fields;
                    message.target_system_id = target;
                    ASSERT_EQ(client.send_message(message), MavlinkDirect::Result::Success);
                }
                ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
                const auto frame = future.get();
                sender.intercept_outgoing_messages_async(nullptr);
                EXPECT_EQ(frame.compat_flags, 0);
                EXPECT_EQ(
                    frame.incompat_flags,
                    (from_server ? MAVLINK_IFLAG_SYSID32 : 0) |
                        (has_target && target > 255 ? MAVLINK_IFLAG_TARGET32 : 0));
                if (has_target) {
                    const auto* meta = mavlink_get_msg_entry(frame.msgid);
                    const auto payload_target =
                        meta->target_system_ofs < frame.len ?
                            static_cast<uint8_t>(_MAV_PAYLOAD(&frame)[meta->target_system_ofs]) :
                            0;
                    EXPECT_EQ(
                        payload_target, target > 255 ? MAVLINK_TARGET_SYSTEM_SENTINEL : target);
                    uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
                    const auto length = mavlink_msg_to_send_buffer(bytes, &frame);
                    EXPECT_EQ(
                        length, frame.len + 12 + (from_server ? 3 : 0) + (target > 255 ? 4 : 0));
                    EXPECT_EQ(
                        mavlink_msg_get_target_sysid(&frame, mavlink_get_msg_entry(frame.msgid)),
                        target);
                    EXPECT_EQ(
                        mavlink_msg_get_target_compid(&frame, mavlink_get_msg_entry(frame.msgid)),
                        std::string(name) == "COMMAND_LONG" ? 1 : 0);
                }
            }
        }
    }
}

TEST(Sysid32, ParametersAndFtpRoundtrip)
{
    Mavsdk ground{Mavsdk::Configuration{0x80000001U, MAV_COMP_ID_MISSIONPLANNER, false}};
    Mavsdk vehicle{Mavsdk::Configuration{0xffffffffU, MAV_COMP_ID_AUTOPILOT1, true}};
    ASSERT_EQ(ground.add_any_connection("udpin://0.0.0.0:17016"), ConnectionResult::Success);
    ASSERT_EQ(vehicle.add_any_connection("udpout://127.0.0.1:17016"), ConnectionResult::Success);
    ParamServer params{vehicle.server_component()};
    FtpServer ftp_server{vehicle.server_component()};
    ASSERT_EQ(params.provide_param_int("TARGET32_TEST", 99), ParamServer::Result::Success);
    auto system = ground.first_autopilot(10.0);
    ASSERT_TRUE(system);
    Param param{*system};
    auto result = param.get_param_int("TARGET32_TEST");
    ASSERT_EQ(result.first, Param::Result::Success);
    EXPECT_EQ(result.second, 99);
    ASSERT_EQ(param.set_param_int("TARGET32_TEST", 101), Param::Result::Success);
    EXPECT_EQ(params.retrieve_param_int("TARGET32_TEST").second, 101);
    const auto directory = test_data_dir() / "target32_ftp";
    ASSERT_TRUE(reset_directories(directory));
    ASSERT_TRUE(create_temp_file(directory / "target32.bin", 4097));
    ftp_server.set_root_dir(directory.string());
    Ftp ftp{*system};
    const auto listing = ftp.list_directory("./");
    ASSERT_EQ(listing.first, Ftp::Result::Success);
    ASSERT_EQ(listing.second.entries.size(), 1U);
    EXPECT_EQ(listing.second.entries.front().name, "target32.bin");
    const auto downloaded = test_data_dir() / "target32_downloaded";
    ASSERT_TRUE(reset_directories(downloaded));
    for (bool burst : {false, true}) {
        auto promise = std::make_shared<std::promise<Ftp::Result>>();
        auto future = promise->get_future();
        ftp.download_async(
            "target32.bin",
            downloaded.string(),
            burst,
            [promise](Ftp::Result result, Ftp::ProgressData) {
                if (result != Ftp::Result::Next) {
                    promise->set_value(result);
                }
            });
        ASSERT_EQ(future.wait_for(std::chrono::seconds(10)), std::future_status::ready);
        ASSERT_EQ(future.get(), Ftp::Result::Success);
        EXPECT_TRUE(are_files_identical(directory / "target32.bin", downloaded / "target32.bin"));
        ASSERT_TRUE(reset_directories(downloaded));
    }
    ASSERT_TRUE(create_temp_file(downloaded / "upload.bin", 4097));
    auto promise = std::make_shared<std::promise<Ftp::Result>>();
    auto future = promise->get_future();
    ftp.upload_async(
        (downloaded / "upload.bin").string(),
        ".",
        [promise](Ftp::Result result, Ftp::ProgressData) {
            if (result != Ftp::Result::Next) {
                promise->set_value(result);
            }
        });
    ASSERT_EQ(future.wait_for(std::chrono::seconds(10)), std::future_status::ready);
    ASSERT_EQ(future.get(), Ftp::Result::Success);
    EXPECT_TRUE(are_files_identical(directory / "upload.bin", downloaded / "upload.bin"));
}

TEST(Sysid32, EightBitPeerStillWorks)
{
    // A system ID that fits in 8 bits must not set any of the new incompat
    // flags, otherwise every peer that predates this feature drops our frames.
    Mavsdk mavsdk_groundstation{Mavsdk::Configuration{ComponentType::GroundStation}};
    Mavsdk mavsdk_autopilot{Mavsdk::Configuration{42, MAV_COMP_ID_AUTOPILOT1, true}};

    ASSERT_EQ(
        mavsdk_groundstation.add_any_connection("udpin://0.0.0.0:17014"),
        ConnectionResult::Success);
    ASSERT_EQ(
        mavsdk_autopilot.add_any_connection("udpout://127.0.0.1:17014"), ConnectionResult::Success);

    auto prom = std::promise<void>();
    auto fut = prom.get_future();
    bool fulfilled = false;

    auto maybe_system = mavsdk_groundstation.first_autopilot(10.0);
    ASSERT_TRUE(maybe_system);
    EXPECT_EQ(maybe_system.value()->get_system_id(), 42);

    auto receiver = MavlinkDirect{maybe_system.value()};
    auto handle = receiver.subscribe_message(
        "HEARTBEAT", [&prom, &fulfilled](MavlinkDirect::MavlinkMessage message) {
            if (!fulfilled && message.system_id == 42) {
                fulfilled = true;
                prom.set_value();
            }
        });

    ASSERT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready);

    receiver.unsubscribe_message(handle);
}

TEST(Sysid32, MissionUploadAndDownload)
{
    Mavsdk ground{Mavsdk::Configuration{0x80000001U, MAV_COMP_ID_MISSIONPLANNER, false}};
    Mavsdk vehicle{Mavsdk::Configuration{0xffffffffU, MAV_COMP_ID_AUTOPILOT1, true}};
    MissionRawServer server{vehicle.server_component()};
    ASSERT_EQ(ground.add_any_connection("udpin://0.0.0.0:17017"), ConnectionResult::Success);
    ASSERT_EQ(vehicle.add_any_connection("udpout://127.0.0.1:17017"), ConnectionResult::Success);
    auto system = ground.first_autopilot(10.0);
    ASSERT_TRUE(system);
    Mission mission{*system};
    Mission::MissionPlan plan;
    for (unsigned i = 0; i < 5; ++i) {
        Mission::MissionItem item{};
        item.latitude_deg = 47.39817 + i * 0.00001;
        item.longitude_deg = 8.545649 + i * 0.00001;
        item.relative_altitude_m = 10.0f;
        item.speed_m_s = 5.0f;
        item.acceptance_radius_m = 2.0f;
        plan.mission_items.push_back(item);
    }
    ASSERT_EQ(mission.upload_mission(plan), Mission::Result::Success);
    const auto downloaded = mission.download_mission();
    ASSERT_EQ(downloaded.first, Mission::Result::Success);
    EXPECT_EQ(downloaded.second, plan);
}

TEST(Sysid32, CommandsRejectAliasedTargetsAndWrongComponents)
{
    Mavsdk vehicle{Mavsdk::Configuration{0x80000001U, MAV_COMP_ID_AUTOPILOT1, false}};
    ActionServer server{vehicle.server_component()};
    server.set_armable(true, true);
    auto calls = std::make_shared<std::atomic<unsigned>>(0);
    auto received = std::make_shared<std::atomic<unsigned>>(0);
    const auto handle = server.subscribe_arm_disarm(
        [calls](ActionServer::Result, ActionServer::ArmDisarm) { ++*calls; });
    ASSERT_EQ(vehicle.add_any_connection("raw://"), ConnectionResult::Success);
    vehicle.intercept_incoming_messages_async([received](mavlink_message_t& frame) {
        if (frame.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
            ++*received;
        }
        return true;
    });
    mavlink_status_t status{};
    auto inject = [&](uint32_t target, uint8_t component) {
        mavlink_message_t frame{};
        mavlink_msg_command_long_pack_status(
            0x80000002U,
            MAV_COMP_ID_MISSIONPLANNER,
            &status,
            &frame,
            target,
            component,
            MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0);
        uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
        const auto length = mavlink_msg_to_send_buffer(bytes, &frame);
        // Feed the actual frame through both receive parsers, including fragmentation.
        for (unsigned i = 0; i < length; ++i) {
            vehicle.pass_received_raw_bytes(reinterpret_cast<const char*>(bytes + i), 1);
        }
    };
    for (uint32_t target : {1U, 255U, 0x81000001U}) {
        inject(target, MAV_COMP_ID_AUTOPILOT1);
    }
    inject(0x80000001U, MAV_COMP_ID_CAMERA);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (received->load() < 4 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(received->load(), 4U);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(calls->load(), 0U);
    for (uint32_t target : {0x80000001U, 0U}) {
        const auto before = calls->load();
        inject(target, 0);
        const auto done = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (calls->load() == before && std::chrono::steady_clock::now() < done) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        EXPECT_EQ(calls->load(), before + 1);
    }
    server.unsubscribe_arm_disarm(handle);
    vehicle.intercept_incoming_messages_async(nullptr);
}

TEST(Sysid32, ForwardedCommandRoundtrip)
{
    Mavsdk ground{Mavsdk::Configuration{0x80000002U, MAV_COMP_ID_MISSIONPLANNER, true}};
    Mavsdk::Configuration router_config{ComponentType::CompanionComputer};
    router_config.set_system_id(0x7fffffffU); // Same low byte as the destination.
    Mavsdk router{router_config};
    Mavsdk vehicle{Mavsdk::Configuration{0xffffffffU, MAV_COMP_ID_AUTOPILOT1, true}};
    ActionServer server{vehicle.server_component()};
    server.set_armable(true, true);
    server.set_disarmable(true, true);
    ASSERT_EQ(ground.add_any_connection("udpin://0.0.0.0:17019"), ConnectionResult::Success);
    ASSERT_EQ(
        router.add_any_connection("udpout://127.0.0.1:17019", ForwardingOption::ForwardingOn),
        ConnectionResult::Success);
    ASSERT_EQ(
        router.add_any_connection("udpin://0.0.0.0:17020", ForwardingOption::ForwardingOn),
        ConnectionResult::Success);
    ASSERT_EQ(vehicle.add_any_connection("udpout://127.0.0.1:17020"), ConnectionResult::Success);
    auto system = ground.first_autopilot(10.0);
    ASSERT_TRUE(system);
    EXPECT_EQ((*system)->get_system_id(), 0xffffffffU);
    Action action{*system};
    EXPECT_EQ(action.arm(), Action::Result::Success);
    EXPECT_EQ(action.disarm(), Action::Result::Success);
}
