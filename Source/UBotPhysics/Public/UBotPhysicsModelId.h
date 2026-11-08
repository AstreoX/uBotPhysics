#pragma once

#include "CoreMinimal.h"
#include "Misc/Crc.h"

namespace UBot::Physics
{
    /** Model names preserve native case sensitivity; FString's default equality is case insensitive. */
    struct FModelIdSetKeyFuncs : DefaultKeyFuncs<FString, false>
    {
        static bool Matches(const FString& A, const FString& B) { return A.Equals(B, ESearchCase::CaseSensitive); }
        static uint32 GetKeyHash(const FString& Key) { return FCrc::StrCrc32(*Key); }
    };

    template<typename ValueType>
    struct TModelIdMapKeyFuncs : TDefaultMapKeyFuncs<FString, ValueType, false>
    {
        static bool Matches(const FString& A, const FString& B) { return A.Equals(B, ESearchCase::CaseSensitive); }
        static uint32 GetKeyHash(const FString& Key) { return FCrc::StrCrc32(*Key); }
    };

    using FModelIdSet = TSet<FString, FModelIdSetKeyFuncs>;

    template<typename ValueType>
    using TModelIdMap = TMap<FString, ValueType, FDefaultSetAllocator, TModelIdMapKeyFuncs<ValueType>>;
}
