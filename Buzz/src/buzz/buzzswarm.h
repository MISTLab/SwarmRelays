#ifndef BUZZSWARM_H
#define BUZZSWARM_H

#include <buzz/buzzdict.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

   /*
    * Forward declaration of the Buzz VM.
    */
   struct buzzvm_s;

   /*
    * Data type for the robot membership data structure.
    */
   typedef buzzdict_t buzzswarm_members_t;

   /*
    * Creates a new swarm membership structure.
    * @return A new swarm membership structure.
    */
   extern buzzswarm_members_t buzzswarm_members_new();

   /*
    * Destroys a swarm membership structure.
    * @param m The swarm membership structure.
    */
   extern void buzzswarm_members_destroy(buzzswarm_members_t* m);

   /*
    * Adds info on the fact that a robot joined a swarm.
    * @param m The swarm membership structure.
    * @param robot The robot id.
    * @param swarm The swarm id.
    */
   extern void buzzswarm_members_join(buzzswarm_members_t m,
                                      uint16_t robot,
                                      uint16_t swarm);

   /*
    * Adds info on the fact that a robot left a swarm.
    * @param m The swarm membership structure.
    * @param robot The robot id.
    * @param swarm The swarm id.
    */
   extern void buzzswarm_members_leave(buzzswarm_members_t m,
                                       uint16_t robot,
                                       uint16_t swarm);

   /*
    * Refreshes the membership information for a robot.
    * The ownership of the passed swarm id list is assumed by
    * this structure. Do not free it.
    * @param m The swarm membership structure.
    * @param robot The robot id.
    * @param swarm The swarms id list.
    */
   extern void buzzswarm_members_refresh(buzzswarm_members_t m,
                                         uint16_t robot,
                                         buzzdarray_t swarms);

   /*
    * Returns 1 if a robot is a member of the given swarm, 0 otherwise.
    * @param m The swarm membership structure.
    * @param robot The robot id.
    * @param swarm The swarm id.
    * @return 1 if a robot is a member of the given swarm, 0 otherwise.
    */
   extern int buzzswarm_members_isrobotin(buzzswarm_members_t m,
                                          uint16_t robot,
                                          uint16_t swarm);

   /*
    * Updates the information in the swarm membership structure.
    * @param m The swarm membership structure.
    */
   extern void buzzswarm_members_update(buzzswarm_members_t m);

   /*
    * Prints the current state of the swarm membership structure.
    * @param stream The stream to use.
    * @param m The swarm membership structure.
    * @param robot The robot id.
    */
   extern void buzzswarm_members_print(FILE* stream,
                                       buzzswarm_members_t m,
                                       uint16_t robot);

   /*
    * Registers the swarm data into the virtual machine.
    * @param vm The Buzz VM state.
    * @return The updated VM state.
    */
   extern int buzzswarm_register(struct buzzvm_s* vm);

   /*
    * Buzz C closure to create a new swarm object.
    * @param vm The Buzz VM state.
    * @return The updated VM state.
    */
   extern int buzzswarm_create(struct buzzvm_s* vm);

   /*
    * Buzz C closure to return the current swarm id or the parent's.
    * @param vm The Buzz VM state.
    * @return The updated VM state.
    */
   extern int buzzswarm_id(struct buzzvm_s* vm);

   /*
    * Buzz C closure to create a new swarm object as a complementary of another.
    * @param vm The Buzz VM state.
    * @return The updated VM state.
    */
   extern int buzzswarm_others(struct buzzvm_s* vm);

   /*
    * Buzz C closure to join a swarm.
    * @param vm The Buzz VM state.
    * @return The updated VM state.
    */
   extern int buzzswarm_join(struct buzzvm_s* vm);

   /*
    * Buzz C closure to leave a swarm.
    * @param vm The Buzz VM state.
    * @return The updated VM state.
    */
   extern int buzzswarm_leave(struct buzzvm_s* vm);

   /*
    * Buzz C closure to check whether the robot is within a swarm.
    * @param vm The Buzz VM state.
    * @return The updated VM state.
    */
   extern int buzzswarm_in(struct buzzvm_s* vm);

   /*
    * Buzz C closure to execute conditionally add a robot to a swarm.
    * @param vm The Buzz VM state.
    * @return The updated VM state.
    */
   extern int buzzswarm_select(struct buzzvm_s* vm);

   /*
    * Buzz C closure to execute a closure if the robot belong to a swarm.
    * @param vm The Buzz VM state.
    * @return The updated VM state.
    */
   extern int buzzswarm_exec(struct buzzvm_s* vm);

   /*
    * Buzz C closure to perform the union of two swarms.
    * @param vm The Buzz VM state.
    * @return The updated VM state.
    */
   extern int buzzswarm_union(struct buzzvm_s* vm);

   /*
    * Buzz C closure to perform the intersection of two swarms.
    * @param vm The Buzz VM state.
    * @return The updated VM state.
    */
   extern int buzzswarm_intersection(struct buzzvm_s* vm);
   
   /*
    * Buzz C closure to perform the difference between two swarms.
    * @param vm The Buzz VM state.
    * @return The updated VM state.
    */
   extern int buzzswarm_difference(struct buzzvm_s* vm);
   
#ifdef __cplusplus
}
#endif

#endif
