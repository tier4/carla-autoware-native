#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace RGLVehicleIgnore
{
    // A ray from a sensor inside a vehicle body crosses many of its faces. RGL libraries that
    // continue a ray behind ignored faces in a loop export rgl_get_ignored_faces_limit (faces one
    // beam sample may pass). Earlier libraries continue with nested traces, which can exceed the
    // ray tracing pipeline's trace depth when a ray crosses a whole vehicle.
    constexpr std::int32_t MinIgnoredFacesLimit = 64;
    // Rejection reason when the loaded library does not pass enough ignored faces.
    inline constexpr const char* IgnoredFacesLimitReason = "rgl_ignored_faces_limit_insufficient";

    // Outcome of asking the loaded library for rgl_get_ignored_faces_limit.
    enum class FacesLimitQuery
    {
        Missing,   // the library does not export the function
        Error,     // the function exists but the call failed
        Available, // the call returned a limit
    };

    inline bool AcceptsIgnoredFacesLimit(FacesLimitQuery Query, std::int32_t Limit)
    {
        return Query == FacesLimitQuery::Available && Limit >= MinIgnoredFacesLimit;
    }

    // key=value field for the rejection line on stdout.
    inline std::string IgnoredFacesLimitField(FacesLimitQuery Query, std::int32_t Limit)
    {
        switch (Query)
        {
        case FacesLimitQuery::Missing: return "ignored_faces_limit=missing";
        case FacesLimitQuery::Error: return "ignored_faces_limit=error";
        case FacesLimitQuery::Available: break;
        }
        return "ignored_faces_limit=" + std::to_string(Limit);
    }

    // Scene managers share RGL's default scene. Allocate on the game thread,
    // across manager lifetimes, so simultaneous worlds cannot reuse an id.
    inline std::int64_t NextSensorId = 1;

    // Zero is reserved for raytrace nodes using the default visibility.
    template <typename Key> class Registry
    {
    public:
        std::int32_t Register(const void* Sensor, const Key& Vehicle)
        {
            for (const auto& B : Bindings)
                if (B.Sensor == Sensor && B.Vehicle == Vehicle) return B.Id;
            Remove(Sensor);
            auto Id = Find(Vehicle);
            if (Id == 0)
            {
                if (NextSensorId > std::numeric_limits<std::int32_t>::max()) return 0;
                Id = static_cast<std::int32_t>(NextSensorId++);
                ++Changes;
            }
            Bindings.push_back({Sensor, Vehicle, Id});
            return Id;
        }
        void Remove(const void* Sensor)
        {
            for (auto It = Bindings.begin(); It != Bindings.end();)
            {
                if (It->Sensor != Sensor) { ++It; continue; }
                const Key Vehicle = It->Vehicle;
                It = Bindings.erase(It);
                if (Find(Vehicle) == 0) ++Changes; // the vehicle's entities lose their id
            }
        }
        std::int32_t Find(const Key& Vehicle) const
        {
            for (const auto& B : Bindings) if (B.Vehicle == Vehicle) return B.Id;
            return 0;
        }
        bool IsEmpty() const { return Bindings.empty(); }
        // Changes whenever a vehicle gains or loses its id, i.e. when entity ids must be refreshed.
        std::uint64_t Revision() const { return Changes; }
    private:
        struct Binding { const void* Sensor; Key Vehicle; std::int32_t Id; };
        std::vector<Binding> Bindings;
        std::uint64_t Changes = 0;
    };

    // Attachment and ownership chains are short. Bounding the walk keeps it free of heap
    // allocation; an entity whose vehicle lies beyond the bound stays visible.
    constexpr std::size_t MaxAncestors = 16;

    // Walk attachment and ownership ancestors, visiting each actor once.
    template <typename Actor, typename Parents, typename Lookup>
    std::int32_t FindSensorId(Actor Start, Parents GetParents, Lookup Find)
    {
        if (!Start) return 0;
        std::array<Actor, MaxAncestors> Pending{};
        std::size_t Count = 0;
        Pending[Count++] = Start;
        for (std::size_t I = 0; I < Count; ++I)
        {
            if (const auto Id = Find(Pending[I])) return Id;
            for (Actor Parent : GetParents(Pending[I]))
                if (Parent && Count < Pending.size() &&
                    std::find(Pending.begin(), Pending.begin() + Count, Parent) == Pending.begin() + Count)
                    Pending[Count++] = Parent;
        }
        return 0;
    }

    // Applied: the id last set on the entity; Wanted: its vehicle's id. Zero means RGL's default
    // (visible to every sensor), which is also the state of a newly created entity.
    struct IgnoreCall
    {
        bool bNeeded;
        std::int32_t SensorId; // argument of rgl_entity_set_ignored_by_sensor
    };
    inline IgnoreCall PlanEntityIgnore(std::int32_t Applied, std::int32_t Wanted, std::int32_t DefaultSensorId)
    {
        if (Applied == Wanted) return {false, 0};
        return {true, Wanted != 0 ? Wanted : DefaultSensorId};
    }

    enum class RefreshStep
    {
        Nothing, // no velocity-distortion sensor and no entity left with an id
        Cached,  // already scanned in this frame with the same registry
        Scan,
    };
    // Entities created after a scan get their id at creation, so one scan per frame covers
    // attachment changes. A registry change within the frame needs a new scan.
    inline RefreshStep PlanRefresh(bool RegistryEmpty, bool AnyApplied, bool Scanned,
                                   std::uint64_t Frame, std::uint64_t ScannedFrame,
                                   std::uint64_t Revision, std::uint64_t ScannedRevision)
    {
        if (RegistryEmpty && !AnyApplied) return RefreshStep::Nothing;
        if (Scanned && Frame == ScannedFrame && Revision == ScannedRevision) return RefreshStep::Cached;
        return RefreshStep::Scan;
    }
}
