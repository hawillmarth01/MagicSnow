import random
import bpy
import math
# from vector3D import *

##################################VECTOR/ COLLISION LIBRARY##################################
#vector op -- length
def length(x_comp, y_comp , z_comp):
    length = math.sqrt((x_comp*x_comp) + (y_comp*y_comp) + (z_comp*z_comp))
    
    return length

#vector op -- normalize
def normalize_tuple(x_comp, y_comp , z_comp):
    mag = length(x_comp, y_comp , z_comp)
    if mag > 0:
        x_comp = x_comp / mag
        y_comp = y_comp / mag
        z_comp = z_comp / mag

    return x_comp, y_comp, z_comp

#vector op -- scalar multiply
def multiply(x_comp, y_comp , z_comp, scalar):
    x_comp *= scalar
    y_comp *= scalar
    z_comp *= scalar

    return x_comp, y_comp, z_comp

#vector op -- add two points
def add(x_comp1, y_comp1 , z_comp1, x_comp2, y_comp2 , z_comp2):
    x_new = x_comp1 + x_comp2
    y_new = y_comp1 + y_comp2
    z_new = z_comp1 + z_comp2

    return x_new, y_new, z_new

#vector op -- subtract two points (first - second)
def subtract(x_comp1, y_comp1 , z_comp1, x_comp2, y_comp2 , z_comp2):
    x_new = x_comp1 - x_comp2
    y_new = y_comp1 - y_comp2
    z_new = z_comp1 - z_comp2

    return x_new, y_new, z_new

#vector op -- distance between two points
def distance(x_comp1, y_comp1 , z_comp1, x_comp2, y_comp2 , z_comp2):
    x = (x_comp1 - x_comp2) * (x_comp1 - x_comp2)
    y = (y_comp1 - y_comp2) * (y_comp1 - y_comp2)
    z = (z_comp1 - z_comp2) * (z_comp1 - z_comp2)
    dist = math.sqrt(x + y + z)

    return dist

#vector op -- clamp to specified length
def clamp(x_comp, y_comp , z_comp, max_len):
    mag = length(x_comp, y_comp , z_comp)
    if mag > max_len:
        shrink_scale = max_len / mag
        x_comp *= shrink_scale
        y_comp *= shrink_scale
        z_comp *= shrink_scale
    
    return x_comp, y_comp, z_comp

#vector op -- dot product
def dot(x_comp1, y_comp1 , z_comp1, x_comp2, y_comp2 , z_comp2):
    dot_prod = (x_comp1 * x_comp2) + (y_comp1 * y_comp2) + (z_comp1 * z_comp2)

    return dot_prod

#vector op -- cross product
def cross(x_comp1, y_comp1, z_comp1, x_comp2, y_comp2 , z_comp2):
    cross_x = (y_comp1 * z_comp2) - (z_comp1 * y_comp2)
    cross_y = (z_comp1 * x_comp2) - (x_comp1 * z_comp2)
    cross_z = (x_comp1 * y_comp2) - (y_comp1 * x_comp2)

    return cross_x, cross_y, cross_z

#vector op -- angle between two vectors
def find_angle(x_comp1, y_comp1, z_comp1, x_comp2, y_comp2 , z_comp2):
    numerator = dot(x_comp1, y_comp1, z_comp1, x_comp2, y_comp2 , z_comp2)
    denominator = (length(x_comp1, y_comp1, z_comp1)) * (length(x_comp2, y_comp2 , z_comp2))
    if denominator == 0:
        denominator = 0.0001
    angle = math.acos(numerator/denominator)

    return angle

#vector op -- rotate vector around z axis using rotation matrix (same as rotating vector in 2D)
def rotateZ(x_comp, y_comp, z_comp, angle):
    new_x = (x_comp * (math.cos(angle))) - (y_comp * (math.sin(angle)))
    new_y = (x_comp * (math.sin(angle))) + (y_comp * (math.cos(angle)))
    new_z = z_comp

    return new_x, new_y, new_z

#vector -- rotate vector around y axis using rotation matrix
def rotateY(x_comp, y_comp, z_comp, angle):
    new_x = (x_comp * (math.cos(angle))) + (z_comp * (math.sin(angle)))
    new_y = y_comp
    new_z = (-x_comp * (math.sin(angle))) + (z_comp * (math.cos(angle)))

    return new_x, new_y, new_z

#collision op -- point in circle
def in_circle(x_comp, y_comp, z_comp, center_x, center_y, center_z, radius):
    curr_dist = distance(x_comp, y_comp, z_comp, center_x, center_y, center_z) #dist between point and center of circle
    if curr_dist < radius + buffer:
        return True
    return False

######################################################################################

# particle attribute storage
particles = []
position = []
velocity = []
acceleration = []
# lives = []

# global variables
num_particles = 500
def_part_size = 0.1 #TUNE
max_speed = 1.5 #TUNE
max_force = 0.03 #TUNE
# dt = 0.1 #TUNE
frame_jump = 9 #TUNE
updates = 50 #number of times update scene, TUNE
#lives = 150 #number of frames a particle can live at full size for

#Boundaries (cylinders, both start at 0,0,0)
outer_radius = 1.25
outer_height = 4.0
inner_radius = 0.5
inner_height = 2.0
buffer = 0.1 #TUNE

#Snowman -- (Estimate)
sm_center = (0.0, 0.0, 7.5)
sm_radius = 8.0
sm_height = 15.0

#generates random position in disk shape
def randomPosition(radius):
    r = radius * random.random() #random scale
    angle = 2 * math.pi * random.random() #random angle
    x_pos = r * math.sin(angle)
    y_pos = r * math.cos(angle)
    
    return x_pos, y_pos

#generates random velocity vector -- (not currently used)
def randomVelocity():
    x_vel = math.cos(0.25*math.pi*random.random()) #cos of random # btwn 0 and PI/4
    y_vel = math.sin(0.25*math.pi*random.random()) #sin of random # btwn 0 and PI/4
    z_vel = math.tan(0.25*math.pi*random.random()) #tan of random # btwn 0 and PI/4

    return x_vel, y_vel, z_vel

#generates random size within bound
def randomSize():
    part_size = 0.03 * random.random()

    return part_size

#sets material settings for particles -- (not currently used, "imported" Blender material instead)
def material():
    material = bpy.data.materials.new(name="ParticleMaterial")
    material.use_nodes = True

    #color
    material.diffuse_color = (0.510, 0.668, 0.813, 1.0)

    #emission shader
    material_output = material.node_tree.nodes.get("Material Output")
    emission = material.node_tree.nodes.new("ShaderNodeEmission")
    emission.inputs["Strength"].default_value = 5.0
    emission.inputs["Color"].default_value = (0.510, 0.668, 0.813, 1) #light blue

    material.node_tree.links.new(material_output.inputs[0], emission.outputs[0])

#initializes arrays, creates boids (cube particles)
def setup():
    #material set-up
    material()

    for p in range(num_particles):
        #add particle to scene (cube)
        bpy.ops.mesh.primitive_cube_add(size=randomSize())
        curr_part = bpy.context.selected_objects[0] #select

        #set material
        mat = bpy.data.materials.get("Particles")
        curr_part.data.materials.append(mat)

        #add to particle list
        particles.append(curr_part)

        #position -- TUNE
        position.append((0.0,0.0,2.6)) #TUNE

        #velocity -- TUNE
        # x_v, y_v, z_v = randomVelocity() #randomize
        # x_v, y_v, z_v = (-9.0,-5.0,-6.0)
        x_v, y_v, z_v = (10.0,5.0,-0.5)
        x_v, y_v, z_v = normalize_tuple(x_v, y_v, z_v) #normalize
        x_v, y_v, z_v = multiply(x_v, y_v, z_v, max_speed) #scale
        velocity.append((x_v,y_v,z_v)) #assign

        acceleration.append((0.0,0.0,0.0)) #initialize to 0

####Boundary Check Plan####:
#Check if outside outer cylinder
#Check if x,y of point position is in perimeter circle and movement direction(dot product)
#If not in circle (i.e. returns false) and moving outwards -->
#Find angle between velocity vector and boundary point... (TBD)
#Find direction of rotation (cross product)
#Rotate velocity a portion of the found angle in found direction
#else, do nothing (is inside of circle)

#Check if inside inner cylinder
#Check if x,y of point position is in perimeter circle and movement direction(dot product)
#If in circle (i.e. returns true) and moving inwards -->
#Repeat collision steps from above
#else, do nothing (is outside of circle)

#Check if below base
#i.e. check if z coordinate is negative (=0) and movement direction(dot product)
#If is equal to or below ground and moving down, reverse direction of velocity?
#else, do nothing

#Can and should go through top
##Note: Will be either or for above scenarios (can't collide with both walls at the same time)
#May need to deal with "corners" of bottom, maybe don't have bottom be an else???

#check if boid is "in bounds" and adjust position and velocity if not 
def borderCheck(p): #takes in index of particle to check location of
    #Note: Assuming radius of particle is negligabley small 
    x = position[p][0]
    y = position[p][1]
    z = position[p][2]

    #Outer Boundary Check
    #(Check if x,y of position are outside of cylinder perimeter for height slices)
    o_hit = False #change to true if goes out of bounds
    o_collision_height = -1.0 #used to store point at which there is a collision
    for s in range(int(outer_height)):
        perim_check = in_circle(x, y, z, 0, 0, outer_height, outer_radius) #returns true if within slice
        if (not(perim_check)): #hitting perimeter (inside)
            move_dir = dot(velocity[p][0], velocity[p][1], velocity[p][2], x, y, s)
            if move_dir >= 0: #CHECK: is moving towards perimeter -- collision
                o_hit = True
                o_collision_height = s #Note: may not be exact bc of buffer w/ in circle check
                break #exit loop, can't intersect more than one boundary
    
    #CHECK if collision occured, adjust velocity accordingly
    if o_hit:
        o_theta = find_angle(velocity[p][0], velocity[p][1], velocity[p][2], x, y, o_collision_height) #angle between velocity and boundary
        o_steer_away = 2.0 * o_theta #TUNE
        velocity[p] = rotateZ(velocity[p][0], velocity[p][1], velocity[p][2], o_steer_away) #rotate velocity vector
        upwards = -20 #TUNE -- angle to keep updward trajectory
        velocity[p] = rotateY(velocity[p][0], velocity[p][1], velocity[p][2], upwards) #rotate velocity vector
    
    #Inner Boundary Check
    #(Check if x,y of position are inside of cylinder perimeter for height slices)
    # i_hit = False #change to true if goes out of bounds
    # i_collision_height = -1.0 #used to store point at which there is a collision
    # for s in range(int(inner_height)):
    #     perim_check = in_circle(x, y, z, 0, 0, inner_height, inner_radius) #returns true if within slice
    #     if (perim_check): #hitting perimeter (outside)
    #         move_dir = dot(velocity[p][0], velocity[p][1], velocity[p][2], x, y, s)
    #         if move_dir >= 0: #CHECK: is moving towards perimeter -- collision
    #             o_hit = True
    #             i_collision_height = s #Note: may not be exact bc of buffer w/ in circle check
    #             break #exit loop, can't intersect more than one boundary
    
    # #CHECK if collision occured, adjust velocity accordingly
    # if i_hit:
    #     i_theta = find_angle(velocity[p][0], velocity[p][1], velocity[p][2], x, y, i_collision_height) #angle between velocity and boundary
    #     i_steer_away = 0.5 * i_theta #TUNE
    #     velocity[p] = rotateZ(velocity[p][0], velocity[p][1], velocity[p][2], i_steer_away) #rotate velocity vector

    #Ground Boundary Check
    #(Check if z is negative)
    if z < 0 or z > 20: #CHECK- should check movement direction?? #below ground level, reverse velocity
        return True

    return False

#obstacle avoidance: snowman --  (not currently used)
#temp: snowman center = (0,0,7.5)
def avoidSnowman(p):
    curr_dist = distance(position[p][0], position[p][1], position[p][2], sm_center[0], sm_center[1], sm_center[2])
    if(curr_dist < sm_radius): #TUNE -- snowman collision
        pos_dir = subtract(position[p][0], position[p][1], position[p][2], sm_center[0], sm_center[1], sm_center[2])
        pos_norm = normalize_tuple(pos_dir[0], pos_dir[1], pos_dir[2])
        move_dir = dot(velocity[p][0], velocity[p][1], velocity[p][2], pos_norm[0], pos_norm[1], pos_norm[2])
        if move_dir >= 0: #CHECK: is moving towards perimeter -- collision
            #adjust position
            pos_change = multiply(pos_norm[0], pos_norm[1], pos_norm[2], (sm_radius * 1.01)) #TUNE -- 1.01 buffer
            position[p] = add(pos_change[0], pos_change[1], pos_change[2], sm_center[0], sm_center[1], sm_center[2])

            #adjust velocity
            vel_norm = multiply(pos_norm[0], pos_norm[1], pos_norm[2], move_dir)
            scale = multiply(vel_norm[0], vel_norm[1], vel_norm[2], (def_part_size + 1)) #TUNE
            velocity[p] = subtract(velocity[p][0], velocity[p][1], velocity[p][2], scale[0], scale[1], scale[2])

#boids force: separation
def separation(p):
    separation_force = (0.0, 0.0, 0.0)
    count = 0
    separation_bound = 5.0 #TUNE

    #iterate thru neighbors
    for n in range(len(particles)):
        curr_dist = distance(position[p][0], position[p][1], position[p][2], position[n][0], position[n][1], position[n][2])
        #separate if too close
        if (curr_dist < separation_bound) and (curr_dist > 0):
            away_vect = subtract(position[p][0], position[p][1], position[p][2], position[n][0], position[n][1], position[n][2]) #vector pointing away from neighbor
            away_vect = normalize_tuple(away_vect[0], away_vect[1], away_vect[2]) #normalize
            away_vect = multiply(away_vect[0], away_vect[1], away_vect[2], (1.0/curr_dist)) #divide by distance

            separation_force = add(separation_force[0], separation_force[1], separation_force[2], away_vect[0], away_vect[1], away_vect[2])
            count += 1
        
    if count > 0:
        separation_force = multiply(separation_force[0], separation_force[1], separation_force[2], (1.0/count))

    return separation_force

#steering force to combine w/ cohesion force
def to_target(targ_pos, p):
    steerTo = subtract(targ_pos[0], targ_pos[1], targ_pos[2], position[p][0], position[p][1], position[p][2])
    steerTo = normalize_tuple(steerTo[0], steerTo[1], steerTo[2])
    steerTo = multiply(steerTo[0], steerTo[1], steerTo[2], max_speed)

    #steering = target - velocity
    steerTo = subtract(steerTo[0], steerTo[1], steerTo[2], velocity[p][0], velocity[p][1], velocity[p][2])
    steerTo = clamp(steerTo[0], steerTo[1], steerTo[2], max_force)

    return steerTo

#boids force: cohesion
def cohesion(p):
    ave_position = (0.0, 0.0, 0.0)
    count = 0
    target_neighbor_dist = 15.0 #TUNE

    #iterate thru neighbors
    for n in range(len(particles)):
        curr_dist = distance(position[p][0], position[p][1], position[p][2], position[n][0], position[n][1], position[n][2])
        #cohere if within target
        if (curr_dist < target_neighbor_dist) and (curr_dist > 0):
            ave_position = add(ave_position[0], ave_position[1], ave_position[2], position[n][0], position[n][1], position[n][2])
            count += 1
    
    if count > 0:
        ave_position = multiply(ave_position[0], ave_position[1], ave_position[2], (1.0/count)) #average
        steer_to_ave = to_target(ave_position, p)
        return steer_to_ave
    else:
        return (0.0,0.0,0.0) 

#boids force: alignment
def alignment(p):
    ave_velocity = (0.0,0.0,0.0)
    count = 0
    target_neighbor_dist = 10.0 #TUNE

    #iterate thru neighbors
    for n in range(len(particles)):
        curr_dist = distance(position[p][0], position[p][1], position[p][2], position[n][0], position[n][1], position[n][2])
        #align if within target
        if (curr_dist < target_neighbor_dist) and (curr_dist > 0):
            ave_velocity = add(ave_velocity[0], ave_velocity[1], ave_velocity[2], velocity[n][0], velocity[n][1], velocity[n][2])
            count += 1
    
    if count > 0:
        ave_velocity = multiply(ave_velocity[0], ave_velocity[1], ave_velocity[2], (1.0/count)) #average
        ave_velocity = clamp(ave_velocity[0], ave_velocity[1], ave_velocity[2], max_force)
    
    return ave_velocity

#update acceleration based on 3 boids forces
def update_acceleration():
    for p in range(len(particles)):
        acceleration[p] = (0.0, 0.0, 0.0)

        #separation
        sep_f = separation(p)
        sep_f = multiply(sep_f[0], sep_f[1], sep_f[2], 1) #weight -- TUNE
        acceleration[p] = add(acceleration[p][0], acceleration[p][1], acceleration[p][2], sep_f[0], sep_f[1], sep_f[2])

        #cohesion
        coh_f = cohesion(p)
        coh_f = multiply(coh_f[0], coh_f[1], coh_f[2], 3) #weight -- TUNE
        acceleration[p] = add(acceleration[p][0], acceleration[p][1], acceleration[p][2], coh_f[0], coh_f[1], coh_f[2])

        #alignment
        ali_f = alignment(p)
        ali_f = multiply(ali_f[0], ali_f[1], ali_f[2], 15) #weight -- TUNE
        acceleration[p] = add(acceleration[p][0], acceleration[p][1], acceleration[p][2], ali_f[0], ali_f[1], ali_f[2])
        
        #extra "wander" force -- TUNE
        tmp_x, tmp_y, tmp_z = 1-random.randint(0,2), 1-random.randint(0,2), 1-random.randint(0,2)
        wf_x, wf_y, wf_z = multiply(tmp_x, tmp_y, tmp_z, 0.5) #TUNE
        acceleration[p] = add(acceleration[p][0], acceleration[p][1], acceleration[p][2], wf_x, wf_y, wf_z)
        
def update_physics():
    update_acceleration()
    for p in range(len(particles)):
        #update velocity
        #temp_accx, temp_accy, temp_accz = multiply(acceleration[p][0], acceleration[p][1], acceleration[p][2], dt)
        velocity[p] = add(velocity[p][0], velocity[p][1], velocity[p][2], acceleration[p][0], acceleration[p][1], acceleration[p][2])

        #keep within max
        mag = length(velocity[p][0], velocity[p][1], velocity[p][2])
        if mag > max_speed:
            temp_velx, temp_vely, temp_velz = normalize_tuple(velocity[p][0], velocity[p][1], velocity[p][2])
            velocity[p] = multiply(temp_velx, temp_vely, temp_velz, max_speed)

        #ensure is within borders, if not, adjust accordingly
        delete_check = borderCheck(p)
    
        #obstacle avoidance -- (not currently implemented, looks better without)
        #avoidSnowman(p)

        #update position
        #temp_posx, temp_posy, temp_posz = multiply(velocity[p][0], velocity[p][1], velocity[p][2], dt)
        position[p] = add(position[p][0], position[p][1], position[p][2], velocity[p][0], velocity[p][1], velocity[p][2])

        #reset acceleration
        acceleration[p] = (0.0,0.0,0.0)
    
    #if needed, delete particle
    if delete_check:
        del particles[p]
        del position[p]
        del velocity[p]
        del acceleration[p]

#updates particles based on frame number
def update(frame_number):
    bpy.context.scene.frame_set(frame_number) #set frame
    
    curr_ind = 0 #current particle index
    for particle in particles:
        particle.location = (position[curr_ind][0], position[curr_ind][1], position[curr_ind][2])
        
        # print(frame_number)
        # if frame_number >= 300: #start to dim/shrink
        #     if (particle.dimensions.x > 0) and (particle.dimensions.y > 0) and (particle.dimensions.z > 0):
        #         print(particle.dimensions)
        #         particle.dimensions = (1.0, 1.0, 1.0)
            #else do nothing

        particle.keyframe_insert(data_path = "location", index = -1) #insert keyframe
        curr_ind += 1

    #update particle attributes for next update
    update_physics() #update pos, vel, accel for particle
    
def main():
    setup()
    frame_number = 128 #hat lands on head here
    for i in range(updates):
        update(frame_number)
        frame_number += frame_jump #move frames forward

if __name__ == '__main__':
    main()