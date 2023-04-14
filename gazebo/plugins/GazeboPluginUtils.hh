#pragma once

static std::string parseString(const std::shared_ptr<const sdf::Element> &sdf,
                               const std::string &element_name) {
  if (sdf->HasElement(element_name)) {
    return sdf->Get<std::string>(element_name);
  } else {
    gzerr << "Failed to parse [" << element_name << "]!";
    exit(-1);
  }
}
