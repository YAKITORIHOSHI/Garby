const { onValueUpdated } = require("firebase-functions/v2/database");
const admin = require("firebase-admin");

// Initialize Firebase Admin with regional Realtime Database URL
admin.initializeApp({
  databaseURL: "https://garby-thesis-default-rtdb.asia-southeast1.firebasedatabase.app"
});

/**
 * Cloud Function v2 trigger listening to state updates at /RASPI/STATES/{stateKey}
 */
exports.onGarbyStateChange = onValueUpdated(
  {
    ref: "/RASPI/STATES/{stateKey}",
    instance: "garby-thesis-default-rtdb"
  },
  async (event) => {
    const stateKey = event.params.stateKey;
    const wasTrue = event.data.before.val() === true;
    const isTrue = event.data.after.val() === true;

    // Only notify when state changes from false -> true
    if (!wasTrue && isTrue) {
      const tokenSnap = await admin.database().ref("APP/fcmToken").get();
      const fcmToken = tokenSnap.val();

      if (!fcmToken) {
        console.log("No active fcmToken found at /APP/fcmToken in database.");
        return;
      }

      let title = "GARBY Alert";
      let body = "GARBY status updated.";

      if (stateKey === "isRunningToPointB") {
        title = "GARBY Navigation Alert";
        body = "GARBY is now running to Point B!";
      } else if (stateKey === "isReadyToReturn") {
        title = "GARBY Status Alert";
        body = "GARBY is now ready to return!";
      } else {
        return; // Ignore unhandled state keys (e.g. launchTime)
      }

      try {
        const response = await admin.messaging().send({
          token: fcmToken,
          notification: { title, body },
          data: {
            event: stateKey,
            [stateKey]: "true"
          },
          android: {
            priority: "high"
          }
        });
        console.log(`[FCM Success] Sent notification for '${stateKey}'. Message ID: ${response}`);
      } catch (error) {
        console.error(`[FCM Error] Failed sending notification for '${stateKey}':`, error);
      }
    }
  }
);
