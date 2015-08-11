/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_NODE_NODE_HPP_INCLUDED
#define UAVCAN_NODE_NODE_HPP_INCLUDED

#include <cassert>
#include <uavcan/error.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/node/abstract_node.hpp>

// High-level functionality available by default
#include <uavcan/protocol/node_status_provider.hpp>

#if !UAVCAN_TINY
# include <uavcan/protocol/data_type_info_provider.hpp>
# include <uavcan/protocol/logger.hpp>
# include <uavcan/protocol/restart_request_server.hpp>
# include <uavcan/protocol/transport_stats_provider.hpp>
#endif

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

namespace uavcan
{
/**
 * This is the top-level node API.
 * A custom node class can be implemented if needed, in which case it shall inherit INode.
 *
 * @tparam MemPoolSize_     Size of memory pool for this node, in bytes.
 *                          Minimum recommended size is 4K * (number of CAN ifaces + 1).
 *                          For simple nodes this number can be reduced.
 *                          For high-traffic nodes the recommended minimum is
 *                          like 16K * (number of CAN ifaces + 1).
 *
 * @tparam OutgoingTransferRegistryStaticEntries    Number of statically allocated objects
 *                                                  to track Transfer ID for outgoing transfers.
 *                                                  Normally it should be equal to expected number of
 *                                                  publishers and service callers, but it's not necessary.
 *                                                  Additional objects for Transfer ID tracking will
 *                                                  be allocated in the memory pool if needed.
 *                                                  Default value is acceptable for any use case.
 */
template <std::size_t MemPoolSize_,
#if UAVCAN_TINY
          unsigned OutgoingTransferRegistryStaticEntries = 0
#else
          unsigned OutgoingTransferRegistryStaticEntries = 10
#endif
          >
class UAVCAN_EXPORT Node : public INode
{
    enum
    {
        MemPoolSize = (MemPoolSize_ < std::size_t(MemPoolBlockSize)) ? std::size_t(MemPoolBlockSize) : MemPoolSize_
    };

    typedef PoolAllocator<MemPoolSize, MemPoolBlockSize> Allocator;

    Allocator pool_allocator_;
    OutgoingTransferRegistry<OutgoingTransferRegistryStaticEntries> outgoing_trans_reg_;
    Scheduler scheduler_;

    NodeStatusProvider proto_nsp_;
#if !UAVCAN_TINY
    DataTypeInfoProvider proto_dtp_;
    Logger proto_logger_;
    RestartRequestServer proto_rrs_;
    TransportStatsProvider proto_tsp_;
#endif

    uint64_t internal_failure_cnt_;
    bool started_;

protected:
    virtual void registerInternalFailure(const char* msg)
    {
        internal_failure_cnt_++;
        UAVCAN_TRACE("Node", "Internal failure: %s", msg);
#if UAVCAN_TINY
        (void)msg;
#else
        (void)getLogger().log(protocol::debug::LogLevel::ERROR, "UAVCAN", msg);
#endif
    }

public:
    Node(ICanDriver& can_driver, ISystemClock& system_clock)
        : outgoing_trans_reg_(pool_allocator_)
        , scheduler_(can_driver, pool_allocator_, system_clock, outgoing_trans_reg_)
        , proto_nsp_(*this)
#if !UAVCAN_TINY
        , proto_dtp_(*this)
        , proto_logger_(*this)
        , proto_rrs_(*this)
        , proto_tsp_(*this)
#endif
        , internal_failure_cnt_(0)
        , started_(false)
    { }

    virtual Allocator& getAllocator() { return pool_allocator_; }

    virtual Scheduler& getScheduler() { return scheduler_; }
    virtual const Scheduler& getScheduler() const { return scheduler_; }

    int spin(MonotonicTime deadline)
    {
        if (started_)
        {
            return INode::spin(deadline);
        }
        return -ErrNotInited;
    }

    int spin(MonotonicDuration duration)
    {
        if (started_)
        {
            return INode::spin(duration);
        }
        return -ErrNotInited;
    }

    int spinOnce()
    {
        if (started_)
        {
            return INode::spinOnce();
        }
        return -ErrNotInited;
    }

    bool isStarted() const { return started_; }

    uint64_t getInternalFailureCount() const { return internal_failure_cnt_; }

    /**
     * Starts the node and publishes uavcan.protocol.NodeStatus immediately.
     * Does not so anything if the node is already started.
     * Once started, the node can't stop.
     * If the node failed to start up, it's recommended to destroy the current node instance and start over.
     * Returns negative error code.
     * @param node_status_transfer_priority Transfer priority that will be used for outgoing NodeStatus messages.
     *                                      Normal priority is used by default.
     */
    int start(const TransferPriority node_status_transfer_priority = TransferPriority::Default);

    /**
     * Sets the node name, e.g. "com.example.product_name". The node name can be set only once.
     * Must be executed before the node is started, otherwise the node will refuse to start up.
     */
    void setName(const char* name) { proto_nsp_.setName(name); }

    /**
     * Node health code helpers.
     */
    void setHealthOk()           { proto_nsp_.setHealthOk(); }
    void setHealthWarning()      { proto_nsp_.setHealthWarning(); }
    void setHealthError()        { proto_nsp_.setHealthError(); }
    void setHealthCritical()     { proto_nsp_.setHealthCritical(); }

    /**
     * Node mode code helpers.
     * Note that INITIALIZATION is the default mode; the application has to manually set it to OPERATIONAL.
     */
    void setModeOperational()    { proto_nsp_.setModeOperational(); }
    void setModeInitialization() { proto_nsp_.setModeInitialization(); }
    void setModeMaintenance()    { proto_nsp_.setModeMaintenance(); }
    void setModeSoftwareUpdate() { proto_nsp_.setModeSoftwareUpdate(); }

    void setModeOfflineAndPublish()
    {
        proto_nsp_.setModeOffline();
        (void)proto_nsp_.forcePublish();
    }

    /**
     * Updates the vendor-specific status code.
     */
    void setVendorSpecificStatusCode(NodeStatusProvider::VendorSpecificStatusCode code)
    {
        proto_nsp_.setVendorSpecificStatusCode(code);
    }

    /**
     * Sets the node version information.
     */
    void setSoftwareVersion(const protocol::SoftwareVersion& version) { proto_nsp_.setSoftwareVersion(version); }
    void setHardwareVersion(const protocol::HardwareVersion& version) { proto_nsp_.setHardwareVersion(version); }

    NodeStatusProvider& getNodeStatusProvider() { return proto_nsp_; }

#if !UAVCAN_TINY
    /**
     * Restart handler can be installed to handle external node restart requests (highly recommended).
     */
    void setRestartRequestHandler(IRestartRequestHandler* handler) { proto_rrs_.setHandler(handler); }

    RestartRequestServer& getRestartRequestServer() { return proto_rrs_; }

    /**
     * Node logging.
     * Logging calls are passed directly into the @ref Logger instance.
     * Type safe log formatting is supported only in C++11 mode.
     * @{
     */
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

    template <typename... Args>
    inline void logDebug(const char* source, const char* format, Args... args)
    {
        (void)proto_logger_.logDebug(source, format, args...);
    }

    template <typename... Args>
    inline void logInfo(const char* source, const char* format, Args... args)
    {
        (void)proto_logger_.logInfo(source, format, args...);
    }

    template <typename... Args>
    inline void logWarning(const char* source, const char* format, Args... args)
    {
        (void)proto_logger_.logWarning(source, format, args...);
    }

    template <typename... Args>
    inline void logError(const char* source, const char* format, Args... args)
    {
        (void)proto_logger_.logError(source, format, args...);
    }

#else

    void logDebug(const char* source, const char* text)   { (void)proto_logger_.logDebug(source, text); }
    void logInfo(const char* source, const char* text)    { (void)proto_logger_.logInfo(source, text); }
    void logWarning(const char* source, const char* text) { (void)proto_logger_.logWarning(source, text); }
    void logError(const char* source, const char* text)   { (void)proto_logger_.logError(source, text); }

#endif
    /**
     * @}
     */

    /**
     * Use this method to configure logging.
     */
    Logger& getLogger() { return proto_logger_; }

#endif  // UAVCAN_TINY
};

// ----------------------------------------------------------------------------

template <std::size_t MemPoolSize_, unsigned OutgoingTransferRegistryStaticEntries>
int Node<MemPoolSize_, OutgoingTransferRegistryStaticEntries>::start(
    const TransferPriority priority)
{
    if (started_)
    {
        return 0;
    }
    GlobalDataTypeRegistry::instance().freeze();

    int res = 0;
    res = proto_nsp_.startAndPublish(priority);
    if (res < 0)
    {
        goto fail;
    }
#if !UAVCAN_TINY
    res = proto_dtp_.start();
    if (res < 0)
    {
        goto fail;
    }
    res = proto_logger_.init();
    if (res < 0)
    {
        goto fail;
    }
    res = proto_rrs_.start();
    if (res < 0)
    {
        goto fail;
    }
    res = proto_tsp_.start();
    if (res < 0)
    {
        goto fail;
    }
#endif
    started_ = res >= 0;
    return res;
fail:
    UAVCAN_ASSERT(res < 0);
    return res;
}

}

#endif // UAVCAN_NODE_NODE_HPP_INCLUDED
