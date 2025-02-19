# The name of the requestor.
string requester

# An id that represents the work order or job associated with this request.
# If the requester reuses the id, it signifies that the same transporter
# from the previous request needs to move to the new destination.
string id

# The destinations for this itinerary.
Destination[] destinations
