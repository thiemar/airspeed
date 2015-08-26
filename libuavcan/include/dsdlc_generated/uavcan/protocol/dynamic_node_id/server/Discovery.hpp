/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /mnt/hgfs/workspace/libuavcan/dsdl/uavcan/protocol/dynamic_node_id/server/390.Discovery.uavcan
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# THIS DEFINITION IS SUBJECT TO CHANGE.
#
# This message is used by allocation servers to find each other's node ID.
# Please refer to the specification for details.
#

#
# This message should be broadcasted by the server at this interval until all other servers are discovered.
#
uint16 BROADCASTING_PERIOD_MS = 1000

#
# Number of servers in the cluster as configured on the sender.
#
uint8 configured_cluster_size

#
# Node ID of servers that are known to the publishing server, including the publishing server itself.
# Capacity of this array defines maximum size of the server cluster.
#
uint8[<=5] known_nodes
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.dynamic_node_id.server.Discovery
saturated uint8 configured_cluster_size
saturated uint8[<=5] known_nodes
******************************************************************************/

#undef configured_cluster_size
#undef known_nodes
#undef BROADCASTING_PERIOD_MS

namespace uavcan
{
namespace protocol
{
namespace dynamic_node_id
{
namespace server
{

template <int _tmpl>
struct UAVCAN_EXPORT Discovery_
{
    typedef const Discovery_<_tmpl>& ParameterType;
    typedef Discovery_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > BROADCASTING_PERIOD_MS;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > configured_cluster_size;
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 5 > known_nodes;
    };

    enum
    {
        MinBitLen
            = FieldTypes::configured_cluster_size::MinBitLen
            + FieldTypes::known_nodes::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::configured_cluster_size::MaxBitLen
            + FieldTypes::known_nodes::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::BROADCASTING_PERIOD_MS >::Type BROADCASTING_PERIOD_MS; // 1000

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::configured_cluster_size >::Type configured_cluster_size;
    typename ::uavcan::StorageType< typename FieldTypes::known_nodes >::Type known_nodes;

    Discovery_()
        : configured_cluster_size()
        , known_nodes()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<51 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 390 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.dynamic_node_id.server.Discovery";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool Discovery_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        configured_cluster_size == rhs.configured_cluster_size &&
        known_nodes == rhs.known_nodes;
}

template <int _tmpl>
bool Discovery_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(configured_cluster_size, rhs.configured_cluster_size) &&
        ::uavcan::areClose(known_nodes, rhs.known_nodes);
}

template <int _tmpl>
int Discovery_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::configured_cluster_size::encode(self.configured_cluster_size, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::known_nodes::encode(self.known_nodes, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Discovery_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::configured_cluster_size::decode(self.configured_cluster_size, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::known_nodes::decode(self.known_nodes, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Discovery_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x821AE2F525F69F21ULL);

    FieldTypes::configured_cluster_size::extendDataTypeSignature(signature);
    FieldTypes::known_nodes::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename Discovery_<_tmpl>::ConstantTypes::BROADCASTING_PERIOD_MS >::Type
    Discovery_<_tmpl>::BROADCASTING_PERIOD_MS = 1000U; // 1000

/*
 * Final typedef
 */
typedef Discovery_<0> Discovery;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::protocol::dynamic_node_id::server::Discovery > _uavcan_gdtr_registrator_Discovery;

}

} // Namespace server
} // Namespace dynamic_node_id
} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::Discovery >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::dynamic_node_id::server::Discovery::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::Discovery >::stream(Stream& s, ::uavcan::protocol::dynamic_node_id::server::Discovery::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "configured_cluster_size: ";
    YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::Discovery::FieldTypes::configured_cluster_size >::stream(s, obj.configured_cluster_size, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "known_nodes: ";
    YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::Discovery::FieldTypes::known_nodes >::stream(s, obj.known_nodes, level + 1);
}

}

namespace uavcan
{
namespace protocol
{
namespace dynamic_node_id
{
namespace server
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::dynamic_node_id::server::Discovery::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::Discovery >::stream(s, obj, 0);
    return s;
}

} // Namespace server
} // Namespace dynamic_node_id
} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_HPP_INCLUDED