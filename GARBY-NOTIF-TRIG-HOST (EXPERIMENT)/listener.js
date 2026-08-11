const admin = require("firebase-admin");
const path = require("path");
const fs = require("fs");
const os = require("os");

const DB_URL = "https://garby-thesis-default-rtdb.asia-southeast1.firebasedatabase.app";

// 1. First check if serviceAccountKey.json exists locally
const serviceAccountPath = path.join(__dirname, "serviceAccountKey.json");

// 2. Otherwise check for CLI credentials in ~/.config/configstore/firebase-tools.json
const cliConfigPath = path.join(os.homedir(), ".config", "configstore", "firebase-tools.json");

let credential;

if (fs.existsSync(serviceAccountPath)) {
  const serviceAccount = require(serviceAccountPath);
  credential = admin.credential.cert(serviceAccount);
  console.log("✅ [GARBY Listener] Authenticated using serviceAccountKey.json");
} else if (fs.existsSync(cliConfigPath)) {
  try {
    const cliConfig = JSON.parse(fs.readFileSync(cliConfigPath, "utf8"));
    const accessToken = cliConfig.tokens?.access_token;
    if (accessToken) {
      credential = {
        getAccessToken: async () => ({
          access_token: accessToken,
          expires_in: 3600
        })
      };
      console.log(`✅ [GARBY Listener] Authenticated automatically using CLI login (${cliConfig.user?.email})`);
    }
  } catch (err) {
    console.warn("⚠️ Could not parse Firebase CLI config:", err.message);
  }
}

if (!credential) {
  console.error("❌ No authentication credential found! Please run 'firebase login' or place 'serviceAccountKey.json' in this folder.");
  process.exit(1);
}

admin.initializeApp({
  credential,
  databaseURL: DB_URL,
  projectId: "garby-thesis"
});

console.log("📡 [GARBY Listener] Connected to Realtime Database:", DB_URL);
console.log("👀 [GARBY Listener] Watching /RASPI/STATES for changes...\n");

const db = admin.database();
const statesRef = db.ref("RASPI/STATES");

let previousStates = {};
let isInitialLoad = true;

statesRef.on("value", async (snapshot) => {
  const currentStates = snapshot.val() || {};

  if (!isInitialLoad) {
    for (const stateKey of Object.keys(currentStates)) {
      const wasTrue = previousStates[stateKey] === true;
      const isTrue = currentStates[stateKey] === true;

      // Notify when state changes from false -> true
      if (!wasTrue && isTrue) {
        console.log(`\n🔔 [GARBY EVENT] ${stateKey} changed from false -> TRUE!`);

        try {
          const tokenSnap = await db.ref("APP/fcmToken").get();
          const fcmToken = tokenSnap.val();

          if (!fcmToken) {
            console.log("⚠️ [FCM] No active fcmToken found in database at APP/fcmToken.");
            continue;
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
            continue;
          }

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

          console.log(`🚀 [FCM SUCCESS] Sent push notification for '${stateKey}'. Message ID: ${response}`);
        } catch (error) {
          console.error(`❌ [FCM ERROR] Failed to send push notification for '${stateKey}':`, error.message);
        }
      }
    }
  } else {
    isInitialLoad = false;
    console.log("🟢 [GARBY Listener] Initial state loaded successfully from database:");
    console.log(currentStates);
  }

  previousStates = { ...currentStates };
}, (errorObject) => {
  console.error("❌ [GARBY Listener Error] Read failed:", errorObject.name, errorObject.message);
});
