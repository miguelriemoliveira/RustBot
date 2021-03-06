# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: NavSatFix.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import Header_pb2 as Header__pb2
import NavSatStatus_pb2 as NavSatStatus__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='NavSatFix.proto',
  package='',
  syntax='proto3',
  serialized_pb=_b('\n\x0fNavSatFix.proto\x1a\x0cHeader.proto\x1a\x12NavSatStatus.proto\"z\n\tNavSatFix\x12\x17\n\x06header\x18\x01 \x01(\x0b\x32\x07.Header\x12\x10\n\x08latitude\x18\x02 \x01(\x01\x12\x11\n\tlongitude\x18\x03 \x01(\x01\x12\x10\n\x08\x61ltitude\x18\x04 \x01(\x01\x12\x1d\n\x06status\x18\x05 \x01(\x0b\x32\r.NavSatStatusb\x06proto3')
  ,
  dependencies=[Header__pb2.DESCRIPTOR,NavSatStatus__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_NAVSATFIX = _descriptor.Descriptor(
  name='NavSatFix',
  full_name='NavSatFix',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='NavSatFix.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='latitude', full_name='NavSatFix.latitude', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude', full_name='NavSatFix.longitude', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='altitude', full_name='NavSatFix.altitude', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status', full_name='NavSatFix.status', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=53,
  serialized_end=175,
)

_NAVSATFIX.fields_by_name['header'].message_type = Header__pb2._HEADER
_NAVSATFIX.fields_by_name['status'].message_type = NavSatStatus__pb2._NAVSATSTATUS
DESCRIPTOR.message_types_by_name['NavSatFix'] = _NAVSATFIX

NavSatFix = _reflection.GeneratedProtocolMessageType('NavSatFix', (_message.Message,), dict(
  DESCRIPTOR = _NAVSATFIX,
  __module__ = 'NavSatFix_pb2'
  # @@protoc_insertion_point(class_scope:NavSatFix)
  ))
_sym_db.RegisterMessage(NavSatFix)


# @@protoc_insertion_point(module_scope)
