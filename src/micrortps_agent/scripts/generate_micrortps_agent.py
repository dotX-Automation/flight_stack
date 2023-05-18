#!/usr/bin/env python3

################################################################################
#
# Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
# Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

# microRTPS Agent code generation script, based on the PX4 Bridge code generation script.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# May 2, 2023

"""
This script can generate the agent code based on a set of topics to send and to receive.
It uses fastrtpsgen to generate the code from the IDL for the topic messages.
The PX4 msg definitions are used to create the IDL used by fastrtpsgen using templates.
"""

import sys
import os
import argparse
import shutil
import px_generate_uorb_topic_files
from uorb_rtps_classifier import Classifier
import subprocess
import glob
import errno
import re

try:
    from six.moves import input
except ImportError as e:
    print("Failed to import six: " + e)
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user six")
    print("")
    sys.exit(1)

try:
    from packaging import version
except ImportError as e:
    print("Failed to import packaging: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user packaging")
    print("")
    sys.exit(1)


def rm_wildcard(pattern):
    """Remove files matching pattern."""
    for f in glob.glob(pattern):
        os.remove(f)


def cp_wildcard(pattern, destdir):
    """Copy files matching pattern to destdir."""
    for f in glob.glob(pattern):
        shutil.copy(f, destdir)


def mkdir_p(dirpath):
    """Create dirpath, if it doesn't exist."""
    try:
        os.makedirs(dirpath)
    except OSError as e:
        if e.errno == errno.EEXIST and os.path.isdir(dirpath):
            pass
        else:
            raise


# Default script argument values
default_agent_out = "src/micrortps_agent"
default_urtps_templates_dir = "src/templates"
default_urtps_topics_file = "src/templates/urtps_bridge_topics.yaml"
default_package_name = px_generate_uorb_topic_files.PACKAGE

# Template file names
uRTPS_AGENT_TOPICS_H_TEMPL_FILE = 'RTPSTopics.hpp.em'
uRTPS_AGENT_TOPICS_SRC_TEMPL_FILE = 'RTPSTopics.cpp.em'
uRTPS_PUBLISHER_SRC_TEMPL_FILE = 'Publisher.cpp.em'
uRTPS_PUBLISHER_H_TEMPL_FILE = 'Publisher.hpp.em'
uRTPS_SUBSCRIBER_SRC_TEMPL_FILE = 'Subscriber.cpp.em'
uRTPS_SUBSCRIBER_H_TEMPL_FILE = 'Subscriber.hpp.em'

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("-i", "--generate-idl", dest='gen_idl',
                        action="store_true", help="Activate generation of IDL files for each message")
    parser.add_argument("-j", "--idl-dir", dest='idl_dir',
                        type=str, help="IDL files directory", default='')
    parser.add_argument("-t", "--topic-msg-dir", dest='msgdir', type=str,
                        help="Message files directory, as path relative to 'msg/'", default="msg")
    parser.add_argument("-q", "--urtps-templates-dir", dest='urtps_templates', type=str,
                        help="Template files directory path, relative to message directory", default=default_urtps_templates_dir)
    parser.add_argument("-y", "--rtps-ids-file", dest='yaml_file', type=str,
                        help="uRTPS Bridge configuration file path, relative to message directory", default=default_urtps_topics_file)
    parser.add_argument("-o", "--agent-outdir", dest='agentdir', type=str,
                        help="Agent output directory", default=default_agent_out)
    parser.add_argument("-f", "--fastrtpsgen-dir", dest='fastrtpsgen', type=str, nargs='?',
                        help="fastrtpsgen installation directory, only needed if fastrtpsgen is not in PATH", default="")
    parser.add_argument("-g", "--fastrtpsgen-include", dest='fastrtpsgen_include', type=str,
                        help="Directories to add to preprocessor include paths of fastrtpsgen", default="")
    parser.add_argument("-r", "--ros2-distro", dest='ros2_distro', type=str, nargs='?',
                        help="ROS 2 distribution", default="")
    parser.add_argument("--delete-tree", dest='del_tree',
                        action="store_true", help="Delete output directories before code generation")

    if len(sys.argv) <= 1:
        parser.print_usage()
        exit(-1)

    # Parse arguments
    args = parser.parse_args()
    del_tree = args.del_tree
    gen_idl = args.gen_idl

    # Message package name
    package = "px4_msgs"

    # Msg files path
    msg_dir = os.path.abspath(args.msgdir)
    px_generate_uorb_topic_files.append_to_include_path(
        {msg_dir}, px_generate_uorb_topic_files.INCL_DEFAULT, package)

    # Agent files output path
    agent_out_dir = os.path.abspath(args.agentdir)

    # IDL files path
    idl_dir = args.idl_dir
    if idl_dir != '':
        idl_dir = os.path.abspath(args.idl_dir)
    else:
        idl_dir = os.path.join(agent_out_dir, "idl")

    if args.fastrtpsgen is None or args.fastrtpsgen == '':
        # Assume fastrtpsgen is in PATH
        fastrtpsgen_path = 'fastrtpsgen'
        for dirname in os.environ['PATH'].split(':'):
            candidate = os.path.join(dirname, 'fastrtpsgen')
            if os.path.isfile(candidate):
                fastrtpsgen_path = candidate
    else:
        # Path to fastrtpsgen is explicitly specified
        if os.path.isdir(args.fastrtpsgen):
            fastrtpsgen_path = os.path.join(
                os.path.abspath(args.fastrtpsgen), 'fastrtpsgen')
        else:
            fastrtpsgen_path = args.fastrtpsgen

    fastrtpsgen_include = args.fastrtpsgen_include
    if fastrtpsgen_include is not None and fastrtpsgen_include != '':
        fastrtpsgen_include = "-I " + os.path.abspath(args.fastrtpsgen_include) + " "

    # Get FastRTPSGen version
    # NOTE: Since Fast-RTPS 1.8.0 release, FastRTPSGen is a separated repository
    # and not included in the Fast-RTPS project.
    # The starting version since this separation is 1.0.0, which follows its own versioning.
    fastrtpsgen_version = version.Version("1.0.0")
    if(os.path.exists(fastrtpsgen_path)):
        try:
            fastrtpsgen_version_out = subprocess.check_output(
                [fastrtpsgen_path, "-version"]).decode("utf-8").strip()[-5:]
        except OSError:
            raise

        try:
            fastrtpsgen_version = version.parse(fastrtpsgen_version_out)
        except version.InvalidVersion:
            raise Exception("'fastrtpsgen -version' returned None or an invalid version")
    else:
        raise Exception(
            "FastRTPSGen not found. Specify the location of fastrtpsgen with the -f flag")

    # Get ROS 2 version
    ros2_distro = ''
    ros_version = os.environ.get('ROS_VERSION')
    if ros_version == '2':
        if args.ros2_distro != '':
            ros2_distro = args.ros2_distro
        else:
            ros2_distro = os.environ.get('ROS_DISTRO')

    # Get FastRTPS version
    fastrtps_version = ''
    fastrtps_version = re.search(r'Version:\s*([\dd.]+)',
                                 subprocess.check_output(
                                     "dpkg -s ros-" + ros2_distro + "-fastrtps 2>/dev/null | grep -i version", shell=True).decode("utf-8").strip()).group(1)

    # Clean build directory if requested
    if del_tree:
        _continue = str(input("\nFiles in " + agent_out_dir + " will be erased, continue? [Y/n] "))
        if _continue == "N" or _continue == "n":
            print("Aborting execution...")
            exit(-1)
        else:
            if os.path.isdir(agent_out_dir):
                shutil.rmtree(agent_out_dir)

    # Remove IDL directory before regenerating
    if os.path.isdir(os.path.join(agent_out_dir, "idl")):
        shutil.rmtree(os.path.join(agent_out_dir, "idl"))

    # uRTPS templates path
    urtps_templates_dir = (args.urtps_templates if os.path.isabs(args.urtps_templates)
                           else os.path.join(msg_dir, args.urtps_templates))

    # Parse YAML file into a map of ids and messages to send and receive
    classifier = (Classifier(os.path.abspath(args.yaml_file), msg_dir) if os.path.isabs(args.yaml_file)
                  else Classifier(os.path.join(msg_dir, args.yaml_file), msg_dir))

    # Generate code for messages to send, i.e., topics to publish
    if classifier.msgs_to_send:
        for msg_file in classifier.msgs_to_send:
            if gen_idl:
                px_generate_uorb_topic_files.generate_idl_file(msg_file, msg_dir, "", idl_dir, urtps_templates_dir,
                                                               package, px_generate_uorb_topic_files.INCL_DEFAULT, fastrtps_version, ros2_distro, classifier.msg_list)
            px_generate_uorb_topic_files.generate_topic_file(msg_file, msg_dir, "", agent_out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_list, fastrtps_version, ros2_distro, uRTPS_PUBLISHER_SRC_TEMPL_FILE)
            px_generate_uorb_topic_files.generate_topic_file(msg_file, msg_dir, "", agent_out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_list, fastrtps_version, ros2_distro, uRTPS_PUBLISHER_H_TEMPL_FILE)
    if classifier.alias_msgs_to_send:
        for msg_file in classifier.alias_msgs_to_send:
            msg_alias = msg_file[0]
            msg_name = msg_file[1]
            if gen_idl:
                px_generate_uorb_topic_files.generate_idl_file(msg_name, msg_dir, msg_alias, idl_dir, urtps_templates_dir,
                                                               package, px_generate_uorb_topic_files.INCL_DEFAULT, fastrtps_version, ros2_distro, classifier.msg_list)
            px_generate_uorb_topic_files.generate_topic_file(msg_name, msg_dir, msg_alias, agent_out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_list, fastrtps_version, ros2_distro, uRTPS_PUBLISHER_SRC_TEMPL_FILE)
            px_generate_uorb_topic_files.generate_topic_file(msg_name, msg_dir, msg_alias, agent_out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_list, fastrtps_version, ros2_distro, uRTPS_PUBLISHER_H_TEMPL_FILE)

    # Generate code for messages to receive, i.e., topics to subscribe
    if classifier.msgs_to_receive:
        for msg_file in classifier.msgs_to_receive:
            if gen_idl:
                px_generate_uorb_topic_files.generate_idl_file(msg_file, msg_dir, "", idl_dir, urtps_templates_dir,
                                                               package, px_generate_uorb_topic_files.INCL_DEFAULT, fastrtps_version, ros2_distro, classifier.msg_list)
            px_generate_uorb_topic_files.generate_topic_file(msg_file, msg_dir, "", agent_out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_list, fastrtps_version, ros2_distro, uRTPS_SUBSCRIBER_SRC_TEMPL_FILE)
            px_generate_uorb_topic_files.generate_topic_file(msg_file, msg_dir, "", agent_out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_list, fastrtps_version, ros2_distro, uRTPS_SUBSCRIBER_H_TEMPL_FILE)
    if classifier.alias_msgs_to_receive:
        for msg_file in classifier.alias_msgs_to_receive:
            msg_alias = msg_file[0]
            msg_name = msg_file[1]
            if gen_idl:
                px_generate_uorb_topic_files.generate_idl_file(msg_name, msg_dir, msg_alias, idl_dir, urtps_templates_dir,
                                                               package, px_generate_uorb_topic_files.INCL_DEFAULT, fastrtps_version, ros2_distro, classifier.msg_list)
            px_generate_uorb_topic_files.generate_topic_file(msg_name, msg_dir, msg_alias, agent_out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_list, fastrtps_version, ros2_distro, uRTPS_SUBSCRIBER_SRC_TEMPL_FILE)
            px_generate_uorb_topic_files.generate_topic_file(msg_name, msg_dir, msg_alias, agent_out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_list, fastrtps_version, ros2_distro, uRTPS_SUBSCRIBER_H_TEMPL_FILE)

    # Generate code for microRTPS Agent core modules
    px_generate_uorb_topic_files.generate_uRTPS_general(classifier.msgs_to_send, classifier.alias_msgs_to_send, classifier.msgs_to_receive, classifier.alias_msgs_to_receive, msg_dir, agent_out_dir,
                                                        urtps_templates_dir, package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_list, fastrtps_version, ros2_distro, uRTPS_AGENT_TOPICS_H_TEMPL_FILE)
    px_generate_uorb_topic_files.generate_uRTPS_general(classifier.msgs_to_send, classifier.alias_msgs_to_send, classifier.msgs_to_receive, classifier.alias_msgs_to_receive, msg_dir, agent_out_dir,
                                                        urtps_templates_dir, package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_list, fastrtps_version, ros2_distro, uRTPS_AGENT_TOPICS_SRC_TEMPL_FILE)

    # Create fastrtpsgen workspace
    mkdir_p(os.path.join(agent_out_dir, "fastrtpsgen"))
    prev_cwd_path = os.getcwd()
    os.chdir(os.path.join(agent_out_dir, "fastrtpsgen"))
    if not glob.glob(os.path.join(idl_dir, "*.idl")):
        raise Exception("No IDL files found in %s" % idl_dir)

    # Set the '-typeros2' option in since it is generating code for interfacing with ROS 2
    gen_ros2_typename = "-typeros2"

    # Determine IDL files to generate code for
    idl_files = []
    for idl_file in glob.glob(os.path.join(idl_dir, "*.idl")):
        if os.path.splitext(os.path.basename(idl_file))[0] in classifier.msg_list:
            idl_files.append(idl_file)

    # Run fastrtpsgen to generate code from IDL files
    try:
        ret = subprocess.check_call(fastrtpsgen_path + " -d " + agent_out_dir + "/fastrtpsgen -example x64Linux2.6gcc " +
                                    gen_ros2_typename + " " + fastrtpsgen_include + " ".join(str(idl_file) for idl_file in idl_files), shell=True)
    except OSError:
        raise

    # Cleanup
    rm_wildcard(os.path.join(agent_out_dir, "fastrtpsgen/*PubSubMain*"))
    rm_wildcard(os.path.join(agent_out_dir, "fastrtpsgen/makefile*"))
    rm_wildcard(os.path.join(agent_out_dir, "fastrtpsgen/*Publisher*"))
    rm_wildcard(os.path.join(agent_out_dir, "fastrtpsgen/*Subscriber*"))
    for f in glob.glob(os.path.join(agent_out_dir, "fastrtpsgen/*.cxx")):
        os.rename(f, f.replace(".cxx", ".cpp"))
    cp_wildcard(os.path.join(agent_out_dir, "fastrtpsgen/*"), agent_out_dir)
    if os.path.isdir(os.path.join(agent_out_dir, "fastrtpsgen")):
        shutil.rmtree(os.path.join(agent_out_dir, "fastrtpsgen"))
    cp_wildcard(os.path.join(urtps_templates_dir,
                             "microRTPS_transport.*"), agent_out_dir)
    os.chdir(prev_cwd_path)

    print("Agent code generated in: " + agent_out_dir)
