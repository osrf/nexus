# The name of the requestor.
string requester

# An id that represents the work order or job associated with this request.
# If the requester reuses the id, it signifies that the same transporter
# from the previous request needs to move to the new destination.
string id

# The name of the destination
string destination

# The name of the source location
string source

# The name of the entity to signal when arriving at the destination, if any.
string signal_destination

# The name of the entity to signal when arriving at the source, if any.
string signal_source

# The items that need to be transported to the destination.
# A json schema may be defined in the future for the contents of this field.
string payload
