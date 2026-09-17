#pragma once

#include <string>
#include <vector>

#include "sura_actions/usv/planar_guidance.hpp"
#include "tinyxml2.h"

namespace sura_actions::usv
{
inline bool loadPath(
  const std::string & file, const std::string & frame,
  std::vector<Pose2D> & output, std::string & error)
{
  tinyxml2::XMLDocument doc;
  if (file.empty() || doc.LoadFile(file.c_str()) != tinyxml2::XML_SUCCESS) {
    error = "Cannot read path XML: " + file;
    return false;
  }
  const auto * root = doc.FirstChildElement("path");
  if (!root) {
    error = "Missing <path> root";
    return false;
  }
  const char * xml_frame = root->Attribute("frame_id");
  if (xml_frame && std::string(xml_frame) != frame) {
    error = "Path frame_id must match configured frame_id: " + frame;
    return false;
  }
  std::vector<Pose2D> parsed;
  for (auto * element = root->FirstChildElement("waypoint"); element;
    element = element->NextSiblingElement("waypoint"))
  {
    int index = -1;
    Pose2D point;
    if (element->QueryIntAttribute("index", &index) != tinyxml2::XML_SUCCESS ||
      index != static_cast<int>(parsed.size()) ||
      element->QueryDoubleAttribute("x", &point.x) != tinyxml2::XML_SUCCESS ||
      element->QueryDoubleAttribute("y", &point.y) != tinyxml2::XML_SUCCESS ||
      element->QueryDoubleAttribute("yaw", &point.theta) != tinyxml2::XML_SUCCESS ||
      !finitePose(point))
    {
      error = "Waypoints require contiguous indices from 0 and finite x, y, yaw";
      return false;
    }
    // Optional AUV z is deliberately ignored.
    point.theta = normalizeAngle(point.theta);
    parsed.push_back(point);
  }
  if (parsed.empty()) {
    error = "Path contains no waypoints";
    return false;
  }
  output = std::move(parsed);
  return true;
}

inline bool savePath(
  const std::string & file, const std::string & frame,
  const std::vector<Pose2D> & points, std::string & error)
{
  if (file.empty() || points.empty()) {
    error = "A non-empty path and filename are required";
    return false;
  }
  tinyxml2::XMLDocument doc;
  auto * root = doc.NewElement("path");
  root->SetAttribute("frame_id", frame.c_str());
  doc.InsertEndChild(root);
  for (size_t i = 0; i < points.size(); ++i) {
    auto * element = doc.NewElement("waypoint");
    element->SetAttribute("index", static_cast<int>(i));
    element->SetAttribute("x", points[i].x);
    element->SetAttribute("y", points[i].y);
    element->SetAttribute("yaw", points[i].theta);
    root->InsertEndChild(element);
  }
  if (doc.SaveFile(file.c_str()) != tinyxml2::XML_SUCCESS) {
    error = "Cannot save path XML: " + file;
    return false;
  }
  return true;
}
}  // namespace sura_actions::usv
