#ifndef __VISION_PARSER_HPP
#define __VISION_PAESER_HPP

#include <common/basic_parser.hpp>
#include <common/CameraProperty.h>
#include <common/datadef.hpp>

extern bool parse(const std::string &filename, common::CameraProperty &camera_pro);
extern bool parse(const std::string &filename, CameraParams &params);
#endif
