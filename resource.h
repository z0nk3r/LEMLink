/**
 * @file resource.h
 *
 * @brief Resource ids the Orbiter launchpad reads.
 *
 * Fixed by Orbiter, not by us: it calls LoadString on the module for 1000 and
 * 1001 to fill the Modules tab. The SDK samples spell them the same way; see
 * Orbitersdk/samples/DX9ExtMFD/resource.h.
 */
#ifndef LEMLINK_RESOURCE_H
#define LEMLINK_RESOURCE_H

#define IDS_INFO 1000   ///< what the module does, shown when it is selected
#define IDS_TYPE 1001   ///< which group it is listed under

#endif /* LEMLINK_RESOURCE_H */
