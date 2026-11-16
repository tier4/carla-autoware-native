// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <cstdint>
#include <cstdlib>
#include <string>

namespace carla {
namespace ros2 {

/// Middleware-neutral QoS profile passed to IPublisherMiddleware::Init /
/// ISubscriberMiddleware::Init when a topic needs QoS different from the
/// vendor defaults (e.g. the Autoware command subscribers require
/// RELIABLE + TRANSIENT_LOCAL to latch the last command).
///
/// This replaces tier4's TopicConfig (data_types.h) in the
/// middleware-decoupled architecture: the domain id is NOT part of the
/// profile (it lives in MiddlewareConfig.h) and the topic suffix is the
/// caller's concern (publishers append suffixes to their base topic name).
///
/// It carries the eight ROS 2 QoS policies (rmw_qos_profile_t): history,
/// depth, reliability, durability, deadline, lifespan, liveliness and the
/// liveliness lease duration. The defaults (RELIABLE / VOLATILE / KEEP_LAST
/// depth 1, no deadline, infinite lifespan, AUTOMATIC liveliness with an
/// infinite lease) mirror the FastDDS DataWriter defaults so passing
/// QosProfile{} to a publisher is equivalent to the QoS-less Init overload.
struct QosProfile {
  enum class Reliability : std::uint8_t { Reliable, BestEffort };
  enum class Durability  : std::uint8_t { Volatile, TransientLocal };
  enum class History     : std::uint8_t { KeepLast, KeepAll };
  /// MANUAL_BY_NODE is left out: ROS 2 deprecated it.
  enum class Liveliness  : std::uint8_t { Automatic, ManualByTopic };

  /// Duration value meaning "infinite" (the DDS default of the deadline, the
  /// lifespan and the liveliness lease duration).
  static constexpr std::int64_t kInfinite = -1;

  Reliability  reliability{Reliability::Reliable};
  Durability   durability{Durability::Volatile};
  History      history{History::KeepLast};
  std::int32_t history_depth{1};
  /// Durations in nanoseconds; kInfinite for none.
  std::int64_t deadline_ns{kInfinite};
  std::int64_t lifespan_ns{kInfinite};
  Liveliness   liveliness{Liveliness::Automatic};
  std::int64_t liveliness_lease_ns{kInfinite};

  bool operator==(const QosProfile &other) const {
    return reliability == other.reliability &&
           durability == other.durability &&
           history == other.history &&
           history_depth == other.history_depth &&
           deadline_ns == other.deadline_ns &&
           lifespan_ns == other.lifespan_ns &&
           liveliness == other.liveliness &&
           liveliness_lease_ns == other.liveliness_lease_ns;
  }
  bool operator!=(const QosProfile &other) const { return !(*this == other); }

  /// True when the deadline, lifespan or liveliness differ from the defaults.
  bool HasTimedPolicies() const {
    return deadline_ns != kInfinite || lifespan_ns != kInfinite ||
           liveliness != Liveliness::Automatic ||
           liveliness_lease_ns != kInfinite;
  }

  /// RELIABLE / TRANSIENT_LOCAL / KEEP_LAST depth 1 — the profile tier4 uses
  /// for the Autoware command subscribers (latched commands survive a late
  /// subscriber join).
  static QosProfile ReliableTransientLocal() {
    QosProfile p;
    p.reliability = Reliability::Reliable;
    p.durability = Durability::TransientLocal;
    p.history = History::KeepLast;
    p.history_depth = 1;
    return p;
  }

  /// RELIABLE / VOLATILE / KEEP_LAST depth 1 — the profile tier4 uses for the
  /// Autoware report publishers.
  static QosProfile ReliableVolatile() {
    QosProfile p;
    p.reliability = Reliability::Reliable;
    p.durability = Durability::Volatile;
    p.history = History::KeepLast;
    p.history_depth = 1;
    return p;
  }

  /// Parses a QoS specification in one of two forms. Returns false (and
  /// leaves `out` untouched) for anything else.
  ///
  /// Short form (tier4, the original sensor QoS settings):
  ///   "<reliable|best_effort>[:<depth>]"
  /// Every other policy keeps its default.
  ///
  /// Key-value form, comma separated, each key at most once:
  ///   reliability=<reliable|best_effort>
  ///   durability=<volatile|transient_local>
  ///   history=<keep_last|keep_all>
  ///   depth=<1..100000>
  ///   deadline=<seconds|inf>
  ///   lifespan=<seconds|inf>
  ///   liveliness=<automatic|manual_by_topic>
  ///   lease=<seconds|inf>          (the liveliness lease duration)
  /// e.g. "reliability=reliable,durability=volatile,history=keep_last,depth=1,
  ///       deadline=inf,lifespan=inf,liveliness=automatic,lease=inf".
  /// A key left out keeps its default, so a caller that must not depend on
  /// the defaults lists all eight. <seconds> is a decimal number greater
  /// than zero with up to nine fractional digits ("0.1", "5"); "inf" means
  /// infinite. Spaces around keys and values are ignored.
  static bool FromString(const std::string &spec, QosProfile &out) {
    QosProfile p;
    const bool ok = spec.find('=') == std::string::npos
        ? ParseShortForm(spec, p)
        : ParseKeyValueForm(spec, p);
    if (ok) {
      out = p;
    }
    return ok;
  }

  /// The key-value form with all eight policies; FromString reads it back to
  /// an equal profile.
  std::string ToString() const {
    std::string s;
    s += "reliability=";
    s += reliability == Reliability::Reliable ? "reliable" : "best_effort";
    s += ",durability=";
    s += durability == Durability::TransientLocal ? "transient_local" : "volatile";
    s += ",history=";
    s += history == History::KeepLast ? "keep_last" : "keep_all";
    s += ",depth=" + std::to_string(history_depth);
    s += ",deadline=" + DurationToString(deadline_ns);
    s += ",lifespan=" + DurationToString(lifespan_ns);
    s += ",liveliness=";
    s += liveliness == Liveliness::Automatic ? "automatic" : "manual_by_topic";
    s += ",lease=" + DurationToString(liveliness_lease_ns);
    return s;
  }

 private:
  static constexpr std::int64_t kNanosPerSecond = 1000000000;
  /// DDS Duration_t keeps the seconds in a 32-bit signed integer.
  static constexpr std::int64_t kMaxSeconds = 2147483647;

  static bool ParseDepth(const std::string &text, std::int32_t &depth) {
    if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
      return false;
    }
    char *end = nullptr;
    const long value = std::strtol(text.c_str(), &end, 10);
    if (end == nullptr || *end != '\0' || value < 1 || value > 100000) {
      return false;
    }
    depth = static_cast<std::int32_t>(value);
    return true;
  }

  static bool ParseShortForm(const std::string &spec, QosProfile &p) {
    const auto colon = spec.find(':');
    const std::string rel = spec.substr(0, colon);
    if (rel == "reliable") {
      p.reliability = Reliability::Reliable;
    } else if (rel == "best_effort") {
      p.reliability = Reliability::BestEffort;
    } else {
      return false;
    }
    if (colon != std::string::npos) {
      return ParseDepth(spec.substr(colon + 1), p.history_depth);
    }
    return true;
  }

  /// "inf", or a decimal number of seconds > 0 with at most nine fractional
  /// digits. Parsed by hand so the result does not depend on the C locale.
  static bool ParseDuration(const std::string &text, std::int64_t &ns) {
    if (text == "inf") {
      ns = kInfinite;
      return true;
    }
    const auto dot = text.find('.');
    const std::string whole = text.substr(0, dot);
    const std::string frac = dot == std::string::npos ? std::string() : text.substr(dot + 1);
    if (whole.empty() || whole.size() > 10 ||
        whole.find_first_not_of("0123456789") != std::string::npos) {
      return false;
    }
    if (dot != std::string::npos &&
        (frac.empty() || frac.size() > 9 ||
         frac.find_first_not_of("0123456789") != std::string::npos)) {
      return false;
    }
    std::int64_t seconds = 0;
    for (const char c : whole) {
      seconds = seconds * 10 + (c - '0');
    }
    if (seconds > kMaxSeconds) {
      return false;
    }
    std::int64_t nanos = 0;
    for (std::size_t i = 0; i < 9; ++i) {
      nanos = nanos * 10 + (i < frac.size() ? frac[i] - '0' : 0);
    }
    const std::int64_t total = seconds * kNanosPerSecond + nanos;
    if (total <= 0) {
      return false;
    }
    ns = total;
    return true;
  }

  static std::string DurationToString(std::int64_t ns) {
    if (ns == kInfinite) {
      return "inf";
    }
    std::string s = std::to_string(ns / kNanosPerSecond);
    std::string frac = std::to_string(ns % kNanosPerSecond);
    frac.insert(0, 9 - frac.size(), '0');
    const auto last = frac.find_last_not_of('0');
    if (last != std::string::npos) {
      s += "." + frac.substr(0, last + 1);
    }
    return s;
  }

  static std::string Trim(const std::string &text) {
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string::npos) {
      return std::string();
    }
    const auto last = text.find_last_not_of(" \t");
    return text.substr(first, last - first + 1);
  }

  static bool ParseKeyValueForm(const std::string &spec, QosProfile &p) {
    unsigned seen = 0u;
    std::size_t begin = 0;
    while (true) {
      const auto comma = spec.find(',', begin);
      const std::string item = spec.substr(
          begin, comma == std::string::npos ? std::string::npos : comma - begin);
      const auto eq = item.find('=');
      if (eq == std::string::npos) {
        return false;
      }
      const std::string key = Trim(item.substr(0, eq));
      const std::string value = Trim(item.substr(eq + 1));
      if (value.empty()) {
        return false;
      }
      unsigned bit = 0u;
      bool ok = false;
      if (key == "reliability") {
        bit = 1u << 0;
        ok = value == "reliable" || value == "best_effort";
        p.reliability = value == "reliable" ? Reliability::Reliable : Reliability::BestEffort;
      } else if (key == "durability") {
        bit = 1u << 1;
        ok = value == "volatile" || value == "transient_local";
        p.durability = value == "volatile" ? Durability::Volatile : Durability::TransientLocal;
      } else if (key == "history") {
        bit = 1u << 2;
        ok = value == "keep_last" || value == "keep_all";
        p.history = value == "keep_last" ? History::KeepLast : History::KeepAll;
      } else if (key == "depth") {
        bit = 1u << 3;
        ok = ParseDepth(value, p.history_depth);
      } else if (key == "deadline") {
        bit = 1u << 4;
        ok = ParseDuration(value, p.deadline_ns);
      } else if (key == "lifespan") {
        bit = 1u << 5;
        ok = ParseDuration(value, p.lifespan_ns);
      } else if (key == "liveliness") {
        bit = 1u << 6;
        ok = value == "automatic" || value == "manual_by_topic";
        p.liveliness = value == "automatic" ? Liveliness::Automatic : Liveliness::ManualByTopic;
      } else if (key == "lease") {
        bit = 1u << 7;
        ok = ParseDuration(value, p.liveliness_lease_ns);
      }
      if (!ok || (seen & bit) != 0u) {
        return false;
      }
      seen |= bit;
      if (comma == std::string::npos) {
        return true;
      }
      begin = comma + 1;
    }
  }
};

} // namespace ros2
} // namespace carla
