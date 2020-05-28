#include <csignal>
#include <string>
#include <vector>

#include <bondcpp/bond.h>
#include <nodelet/nodelet.h>
#include <nodelet/loader.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <unique_id/unique_id.h>

/*
 * This `nodelet\_or\_node` node allows launching nodelets either by loading them or by running them
 * standalone. In this, it is similar to `nodelet load` and `nodelet standalone` from the
 * [nodelet](https://wiki.ros.org/nodelet) package.
 *
 * However, `nodelet_or_node` makes it easy to differentiate the load/standalone mode by just
 * providing (or not) the nodelet manager name. This helps writing nice interfaces which easily
 * allow the user to choose whether the node should run in a nodelet manager or standalone.
 *
 * A nodelet loaded into a manager using `nodelet_or_node` can be unloaded using all standard ways
 * for unloading nodelets.
 *
 * See the readme for examples.
 *
 * A big part of this file is based on the code of nodelet/src/nodelet.cpp from nodelet package.
 * The code was modernized, but the basic functionality related to loading/running standalone
 * remained the same.
 */

sig_atomic_t volatile requestShutdown = 0;
void nodeletLoaderSigIntHandler(int)
{
    requestShutdown = 1;
}

// Shutdown can be triggered externally by an XML-RPC call, this is how "rosnode kill"
// works. When shutting down a "nodelet load" we always want to unload the nodelet
// before shutting down our ROS comm channels, so we override the default roscpp
// handler for a "shutdown" XML-RPC call.
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    int num_params = 0;
    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
        num_params = params.size();
    if (num_params > 1)
    {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
        requestShutdown = 1;
    }

    result = ros::xmlrpc::responseInt(1, "", 0);
}

class NodeletInterface
{
public:

  /** \brief Unload the nodelet */
  bool unloadNodelet (const std::string& name, const std::string& manager)
  {
      ROS_INFO("Unloading nodelet '%s' from manager '%s'", name.c_str(), manager.c_str());

      const auto serviceName = manager + "/unload_nodelet";
      // Check if the service exists and is available
      if (!ros::service::exists(serviceName, true))
      {
          // Probably the manager has shut down already, which is fine
          ROS_WARN("Couldn't find service %s, perhaps the manager is already shut down",
                   serviceName.c_str());
          return false;
      }

      auto client = this->publicNodeHandle.serviceClient<nodelet::NodeletUnload>(serviceName);

      // Call the service
      nodelet::NodeletUnload srv;
      srv.request.name = name;
      if (!client.call(srv))
      {
          // Maybe service shut down in the meantime, which isn't an error
          if (ros::service::exists(serviceName, false))
              ROS_FATAL("Failed to unload nodelet '%s' from manager '%s'", name.c_str(),
                  manager.c_str());
          return false;
      }
      return true;
  }

  /** \brief Load the nodelet */
  bool loadNodelet (const std::string& name, const std::string& type, const std::string& manager,
      const std::vector<std::string>& args, const std::string& bond_id)
  {
      const auto& remappings = ros::names::getRemappings();
      std::vector<std::string> sources(remappings.size());
      std::vector<std::string> targets(remappings.size());

      ROS_INFO("Loading nodelet %s of type %s to manager %s with the following remappings:",
          name.c_str(), type.c_str(), manager.c_str());
      size_t i = 0;
      for (const auto& remap : remappings)
      {
          sources[i] = remap.first;
          targets[i] = remap.second;
          ROS_INFO("%s -> %s", sources[i].c_str(), targets[i].c_str());
          ++i;
      }

      // Get and set the parameters
      XmlRpc::XmlRpcValue param;
      const auto& nodeName = ros::this_node::getName();
      this->publicNodeHandle.getParam(nodeName, param);
      this->publicNodeHandle.setParam(name, param);

      const auto service_name = manager + "/load_nodelet";

      // Wait until the service is advertised
      ROS_DEBUG("Waiting for service %s to be available...", service_name.c_str());
      auto client = this->publicNodeHandle.serviceClient<nodelet::NodeletLoad>(service_name);
      client.waitForExistence();

      // Call the service
      nodelet::NodeletLoad srv;
      srv.request.name = name;
      srv.request.type = type;
      srv.request.remap_source_args = sources;
      srv.request.remap_target_args = targets;
      srv.request.my_argv = args;
      srv.request.bond_id = bond_id;
      if (!client.call(srv))
      {
          ROS_FATAL("Failed to load nodelet '%s' of type '%s' to manager '%s'", name.c_str(),
              type.c_str(), manager.c_str());
          return false;
      }

      return true;
  }

private:
  ros::NodeHandle publicNodeHandle;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nodelet_or_node");

    std::vector<std::string> args;
    ros::removeROSArgs(argc, argv, args);

    args.erase(args.begin());  // program name

    if (args.empty())
        return 1;

    const auto pkgAndNodelet = args[0];
    args.erase(args.begin());

    if (pkgAndNodelet.find('/') == std::string::npos)
    {
        ROS_ERROR("First argument of nodelet_or_node has to be pkg/NodeletName, but %s was given", pkgAndNodelet.c_str());
        return 2;
    }

    const auto& nodeName = ros::this_node::getName();

    if (args.empty())  // nodelet manager was not given
    {
        nodelet::Loader nodelet;
        const nodelet::M_string noRemap;  // remappings are already done in init()
        const nodelet::V_string noArgv;

        ROS_INFO("Loading standalone nodelet of type '%s' into name '%s'", pkgAndNodelet.c_str(),
            nodeName.c_str());

        if (!nodelet.load(nodeName, pkgAndNodelet, noRemap, noArgv))
        {
            ROS_ERROR("Failed to launch standalone nodelet %s.", pkgAndNodelet.c_str());
            return 3;
        }

        ROS_DEBUG("Successfully loaded standalone nodelet of type '%s' into name '%s'",
            pkgAndNodelet.c_str(), nodeName.c_str());

        ros::spin();
    }
    else
    {
        NodeletInterface nodeletInterface;
        const auto manager = args[0];
        args.erase(args.begin());

        std::string bondId;
        if (args.empty() || args[0] != "--no-bond")
            bondId = nodeName + "_" + unique_id::toHexString(unique_id::fromRandom());
        else
            args.erase(args.begin());
        bond::Bond bond(manager + "/bond", bondId);

        if (!nodeletInterface.loadNodelet(nodeName, pkgAndNodelet, manager, args, bondId))
            return 4;

        // Override default exit handlers for roscpp
        signal(SIGINT, nodeletLoaderSigIntHandler);
        ros::XMLRPCManager::instance()->unbind("shutdown");
        ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

        if (!bondId.empty())
            bond.start();

        ros::AsyncSpinner spinner(1);
        spinner.start();
        while (!requestShutdown)
        {
            if (!bondId.empty() && bond.isBroken())
            {
                ROS_INFO("Bond broken, exiting");
                goto shutdown;
            }
            ros::WallDuration(0.1).sleep();
        }

        // Attempt to unload the nodelet before shutting down ROS
        nodeletInterface.unloadNodelet(nodeName, manager);
        if (!bondId.empty())
            bond.breakBond();

        shutdown:
        ros::shutdown();
    }

    return 0;
}