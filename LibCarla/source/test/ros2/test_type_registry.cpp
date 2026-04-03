// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <gtest/gtest.h>
#include <string>
#include <unordered_map>
#include <functional>

// Minimal stand-in for type registration pattern test
namespace {
    using TypeCreator = std::function<bool()>;
    std::unordered_map<std::string, TypeCreator>& GetRegistry() {
        static std::unordered_map<std::string, TypeCreator> registry;
        return registry;
    }
    void RegisterType(const std::string& name, TypeCreator creator) {
        GetRegistry()[name] = std::move(creator);
    }
    bool HasType(const std::string& name) {
        return GetRegistry().count(name) > 0;
    }
}

TEST(TypeRegistry, RegisterAndLookup) {
    RegisterType("sensor_msgs::msg::PointCloud2", []() { return true; });
    RegisterType("std_msgs::msg::String", []() { return true; });

    ASSERT_TRUE(HasType("sensor_msgs::msg::PointCloud2"));
    ASSERT_TRUE(HasType("std_msgs::msg::String"));
    ASSERT_FALSE(HasType("unknown::msg::Type"));
}

TEST(TypeRegistry, EmptyNameReturnsFalse) {
    ASSERT_FALSE(HasType(""));
}
