/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /mnt/hgfs/workspace/libuavcan/dsdl/uavcan/Timestamp.uavcan
 */

#ifndef UAVCAN_TIMESTAMP_HPP_INCLUDED
#define UAVCAN_TIMESTAMP_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Global timestamp in hectomicroseconds (1 = 100 usec), 6 bytes.
#

uint48 UNKNOWN = 0
uint48 USEC_PER_LSB = 100  # Timestamp resolution
truncated uint48 husec     # Hectomicroseconds (10^-4)
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.Timestamp
truncated uint48 husec
******************************************************************************/

#undef husec
#undef UNKNOWN
#undef USEC_PER_LSB

namespace uavcan
{

template <int _tmpl>
struct UAVCAN_EXPORT Timestamp_
{
    typedef const Timestamp_<_tmpl>& ParameterType;
    typedef Timestamp_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 48, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > UNKNOWN;
        typedef ::uavcan::IntegerSpec< 48, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > USEC_PER_LSB;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 48, ::uavcan::SignednessUnsigned, ::uavcan::CastModeTruncate > husec;
    };

    enum
    {
        MinBitLen
            = FieldTypes::husec::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::husec::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::UNKNOWN >::Type UNKNOWN; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::USEC_PER_LSB >::Type USEC_PER_LSB; // 100

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::husec >::Type husec;

    Timestamp_()
        : husec()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<48 == MaxBitLen>::check();
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
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "uavcan.Timestamp";
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
bool Timestamp_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        husec == rhs.husec;
}

template <int _tmpl>
bool Timestamp_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(husec, rhs.husec);
}

template <int _tmpl>
int Timestamp_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::husec::encode(self.husec, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Timestamp_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::husec::decode(self.husec, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Timestamp_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x9231EF4D9AD8B388ULL);

    FieldTypes::husec::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename Timestamp_<_tmpl>::ConstantTypes::UNKNOWN >::Type
    Timestamp_<_tmpl>::UNKNOWN = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename Timestamp_<_tmpl>::ConstantTypes::USEC_PER_LSB >::Type
    Timestamp_<_tmpl>::USEC_PER_LSB = 100U; // 100

/*
 * Final typedef
 */
typedef Timestamp_<0> Timestamp;

// No default registration

} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::Timestamp >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::Timestamp::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::Timestamp >::stream(Stream& s, ::uavcan::Timestamp::ParameterType obj, const int level)
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
    s << "husec: ";
    YamlStreamer< ::uavcan::Timestamp::FieldTypes::husec >::stream(s, obj.husec, level + 1);
}

}

namespace uavcan
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::Timestamp::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::Timestamp >::stream(s, obj, 0);
    return s;
}

} // Namespace uavcan

#endif // UAVCAN_TIMESTAMP_HPP_INCLUDED