package net.phys2d.raw;

/**
 * A description of class that can recieve notifications of collisions
 * within a <code>CollisionSpace</code>
 * 
 * @author Kevin Glass
 */
public interface CollisionListener {
	/**
	 * Notification that a collision occured
	 * 
	 * @param event The describing the collision
	 */
	public void collisionOccured(CollisionEvent event);
        
        /**
         * Notification that a separation occured
         * @param event describing the separation
         */
        public void separationOccured(CollisionEvent event);
}
