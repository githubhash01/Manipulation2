#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to cw1_world_spawner__srv__TaskSetup_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TaskSetup_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub task_index: i8,

}



impl Default for TaskSetup_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::TaskSetup_Request::default())
  }
}

impl rosidl_runtime_rs::Message for TaskSetup_Request {
  type RmwMsg = super::srv::rmw::TaskSetup_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_index: msg.task_index,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      task_index: msg.task_index,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      task_index: msg.task_index,
    }
  }
}


// Corresponds to cw1_world_spawner__srv__TaskSetup_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TaskSetup_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for TaskSetup_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::TaskSetup_Response::default())
  }
}

impl rosidl_runtime_rs::Message for TaskSetup_Response {
  type RmwMsg = super::srv::rmw::TaskSetup_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to cw1_world_spawner__srv__Task1Service_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Task1Service_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub object_loc: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_loc: geometry_msgs::msg::PointStamped,

}



impl Default for Task1Service_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Task1Service_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Task1Service_Request {
  type RmwMsg = super::srv::rmw::Task1Service_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        object_loc: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.object_loc)).into_owned(),
        goal_loc: geometry_msgs::msg::PointStamped::into_rmw_message(std::borrow::Cow::Owned(msg.goal_loc)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        object_loc: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.object_loc)).into_owned(),
        goal_loc: geometry_msgs::msg::PointStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_loc)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      object_loc: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.object_loc),
      goal_loc: geometry_msgs::msg::PointStamped::from_rmw_message(msg.goal_loc),
    }
  }
}


// Corresponds to cw1_world_spawner__srv__Task1Service_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Task1Service_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for Task1Service_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Task1Service_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Task1Service_Response {
  type RmwMsg = super::srv::rmw::Task1Service_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to cw1_world_spawner__srv__Task2Service_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Task2Service_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub basket_locs: Vec<geometry_msgs::msg::PointStamped>,

}



impl Default for Task2Service_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Task2Service_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Task2Service_Request {
  type RmwMsg = super::srv::rmw::Task2Service_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        basket_locs: msg.basket_locs
          .into_iter()
          .map(|elem| geometry_msgs::msg::PointStamped::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        basket_locs: msg.basket_locs
          .iter()
          .map(|elem| geometry_msgs::msg::PointStamped::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      basket_locs: msg.basket_locs
          .into_iter()
          .map(geometry_msgs::msg::PointStamped::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to cw1_world_spawner__srv__Task2Service_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Task2Service_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub basket_colours: Vec<std::string::String>,

}



impl Default for Task2Service_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Task2Service_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Task2Service_Response {
  type RmwMsg = super::srv::rmw::Task2Service_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        basket_colours: msg.basket_colours
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        basket_colours: msg.basket_colours
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      basket_colours: msg.basket_colours
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
    }
  }
}


// Corresponds to cw1_world_spawner__srv__Task3Service_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Task3Service_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for Task3Service_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Task3Service_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Task3Service_Request {
  type RmwMsg = super::srv::rmw::Task3Service_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to cw1_world_spawner__srv__Task3Service_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Task3Service_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for Task3Service_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Task3Service_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Task3Service_Response {
  type RmwMsg = super::srv::rmw::Task3Service_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}






#[link(name = "cw1_world_spawner__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__cw1_world_spawner__srv__TaskSetup() -> *const std::ffi::c_void;
}

// Corresponds to cw1_world_spawner__srv__TaskSetup
#[allow(missing_docs, non_camel_case_types)]
pub struct TaskSetup;

impl rosidl_runtime_rs::Service for TaskSetup {
    type Request = TaskSetup_Request;
    type Response = TaskSetup_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__cw1_world_spawner__srv__TaskSetup() }
    }
}




#[link(name = "cw1_world_spawner__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__cw1_world_spawner__srv__Task1Service() -> *const std::ffi::c_void;
}

// Corresponds to cw1_world_spawner__srv__Task1Service
#[allow(missing_docs, non_camel_case_types)]
pub struct Task1Service;

impl rosidl_runtime_rs::Service for Task1Service {
    type Request = Task1Service_Request;
    type Response = Task1Service_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__cw1_world_spawner__srv__Task1Service() }
    }
}




#[link(name = "cw1_world_spawner__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__cw1_world_spawner__srv__Task2Service() -> *const std::ffi::c_void;
}

// Corresponds to cw1_world_spawner__srv__Task2Service
#[allow(missing_docs, non_camel_case_types)]
pub struct Task2Service;

impl rosidl_runtime_rs::Service for Task2Service {
    type Request = Task2Service_Request;
    type Response = Task2Service_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__cw1_world_spawner__srv__Task2Service() }
    }
}




#[link(name = "cw1_world_spawner__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__cw1_world_spawner__srv__Task3Service() -> *const std::ffi::c_void;
}

// Corresponds to cw1_world_spawner__srv__Task3Service
#[allow(missing_docs, non_camel_case_types)]
pub struct Task3Service;

impl rosidl_runtime_rs::Service for Task3Service {
    type Request = Task3Service_Request;
    type Response = Task3Service_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__cw1_world_spawner__srv__Task3Service() }
    }
}


