    const canvas = document.getElementById("mapCanvas");
    const ctx = canvas.getContext("2d");
    const state = {
      map: null,
      mapImage: null,
      latest: null,
      liveMapVersion: -1,
      selectedMapId: null,
      fileMapVersion: -1,
      maps: [],
      annotations: [],
      tasks: [],
      selectedMapStatus: null,
      sessionId: null,
      markDraft: null,
      markDraftSource: "map_click",
      localizeDraft: null,
      markPointer: null,
      panPointer: null,
      robotDisplayPose: null,
      view: {
        zoom: 1,
        panX: 0,
        panY: 0,
        panMode: false
      },
      followRobot: true,
      preflight: null,
      taskReadiness: null,
      lastRelocalizationStamp: null,
      relocalizationApiLogUntil: 0,
      activeTaskLogUntil: 0,
      lastTasksRefreshAt: 0,
      lastTasksPayload: null,
      loadingTasks: false,
      scanOverlay: true,
      mapModeLabel: "实时 /map",
    };
    window.m20proDebug = {
      snapshot() {
        return {
          latest: state.latest,
          selectedMapId: state.selectedMapId,
          maps: state.maps,
          annotations: state.annotations,
          tasks: state.tasks,
          selectedMapStatus: state.selectedMapStatus,
          lastTasksPayload: state.lastTasksPayload,
          taskReadiness: state.taskReadiness,
          preflight: state.preflight,
          followRobot: state.followRobot,
          view: state.view
        };
      },
      fieldSnapshot() {
        return typeof buildFieldSnapshot === "function" ? buildFieldSnapshot() : null;
      }
    };
    const manualPointTypeNames = {
      transition: "过渡点",
      task: "任务点",
      charge: "充电点"
    };
    const manualTypeByUiType = {
      patrol: "task",
      task: "task",
      stair_entry: "transition",
      stair_switch: "transition",
      stair_exit: "transition",
      transition: "transition",
      charge: "charge"
    };
    const defaultByManualType = {
      transition: { dwell: 0, gait: 12, speed: 1, manner: 0, obsMode: 0, navMode: 0 },
      task: { dwell: 5, gait: 12, speed: 1, manner: 0, obsMode: 0, navMode: 1 },
      charge: { dwell: 0, gait: 12, speed: 1, manner: 0, obsMode: 0, navMode: 1 }
    };
    const typeNames = {
      patrol: "巡检点",
      stair_entry: "步态切换点",
      stair_switch: "楼层切换点",
      stair_exit: "出楼梯点",
      charge: "充电点",
      transition: "过渡点"
    };
    const typeColors = {
      patrol: "#0f6bff",
      stair_entry: "#f97316",
      stair_switch: "#7c3aed",
      stair_exit: "#0891b2",
      charge: "#15803d",
      transition: "#b45309"
    };
    const cameraViewers = {
      front: { active: false, token: 0, objectUrl: null, abortController: null, latestPayload: null, renderScheduled: false },
      rear: { active: false, token: 0, objectUrl: null, abortController: null, latestPayload: null, renderScheduled: false }
    };

	    function $(id) { return document.getElementById(id); }
	    function text(value) { return value === null || value === undefined || value === "" ? "-" : String(value); }
	    function fmtNumber(value, digits = 2) { return Number.isFinite(value) ? value.toFixed(digits) : "-"; }
	    function clearVideoFrame(name) {
	      const img = $(`${name}Video`);
	      const viewer = cameraViewers[name];
	      if (!img || !viewer) return;
	      img.removeAttribute("src");
	      if (viewer.abortController) {
	        viewer.abortController.abort();
	        viewer.abortController = null;
	      }
	      viewer.latestPayload = null;
	      viewer.renderScheduled = false;
	      if (viewer.objectUrl) {
	        URL.revokeObjectURL(viewer.objectUrl);
	        viewer.objectUrl = null;
	      }
	    }
	    function parseMjpegHeaders(headerText) {
	      const headers = {};
	      for (const line of headerText.split("\r\n")) {
	        const index = line.indexOf(":");
	        if (index <= 0) continue;
	        headers[line.slice(0, index).trim().toLowerCase()] = line.slice(index + 1).trim();
	      }
	      return headers;
	    }
	    async function displayVideoFrame(img, viewer, payload) {
	      const nextUrl = URL.createObjectURL(new Blob([payload], { type: "image/jpeg" }));
	      const oldUrl = viewer.objectUrl;
	      viewer.objectUrl = nextUrl;
	      img.src = nextUrl;
	      if (oldUrl) URL.revokeObjectURL(oldUrl);
	    }
	    function queueLatestVideoFrame(img, viewer, token, payload) {
	      viewer.latestPayload = payload;
	      if (viewer.renderScheduled) return;
	      viewer.renderScheduled = true;
	      requestAnimationFrame(() => {
	        viewer.renderScheduled = false;
	        const latestPayload = viewer.latestPayload;
	        viewer.latestPayload = null;
	        if (!viewer.active || viewer.token !== token || !latestPayload) return;
	        displayVideoFrame(img, viewer, latestPayload);
	      });
	    }
	    async function pumpVideoFrames(name, token) {
	      const img = $(`${name}Video`);
	      const viewer = cameraViewers[name];
	      if (!img || !viewer) return;
	      const source = img.dataset.src;
	      while (viewer.active && viewer.token === token && source) {
	        const abortController = new AbortController();
	        viewer.abortController = abortController;
	        try {
	          const separator = source.includes("?") ? "&" : "?";
	          const response = await fetch(`${source}${separator}ts=${Date.now()}`, {
	            cache: "no-store",
	            signal: abortController.signal
	          });
	          if (!response.ok) throw new Error(`HTTP ${response.status}`);
	          if (!response.body) throw new Error("stream unavailable");
	          const reader = response.body.getReader();
	          let buffer = new Uint8Array(0);
	          while (viewer.active && viewer.token === token) {
	            const { value, done } = await reader.read();
	            if (done) break;
	            if (!value) continue;
	            const merged = new Uint8Array(buffer.length + value.length);
	            merged.set(buffer);
	            merged.set(value, buffer.length);
	            buffer = merged;
	            while (viewer.active && viewer.token === token) {
	              const headerEnd = findBytePattern(buffer, [13, 10, 13, 10]);
	              if (headerEnd < 0) {
	                if (buffer.length > 2000000) buffer = buffer.slice(buffer.length - 4096);
	                break;
	              }
	              const headerText = new TextDecoder("latin1").decode(buffer.slice(0, headerEnd));
	              const headers = parseMjpegHeaders(headerText);
	              const contentLength = Number(headers["content-length"]);
	              if (!Number.isFinite(contentLength) || contentLength <= 0) {
	                const nextBoundary = findBytePattern(buffer.slice(Math.max(1, headerEnd)), [45, 45, 102, 114, 97, 109, 101]);
	                if (nextBoundary < 0) break;
	                buffer = buffer.slice(Math.max(1, headerEnd) + nextBoundary);
	                continue;
	              }
	              const frameStart = headerEnd + 4;
	              const frameEnd = frameStart + contentLength;
	              if (buffer.length < frameEnd + 2) break;
		              const payload = buffer.slice(frameStart, frameEnd);
		              buffer = buffer.slice(frameEnd + 2);
		              if (payload.length > 2 && payload[0] === 0xff && payload[1] === 0xd8) {
		                queueLatestVideoFrame(img, viewer, token, payload);
		              }
	            }
	          }
	        } catch (err) {
	          if (!viewer.active || viewer.token !== token || err.name === "AbortError") break;
	          await new Promise(resolve => setTimeout(resolve, 500));
	        } finally {
	          if (viewer.abortController === abortController) viewer.abortController = null;
	        }
	      }
	      if (viewer.token === token && !viewer.active) clearVideoFrame(name);
	    }
	    function findBytePattern(buffer, pattern) {
	      outer:
	      for (let i = 0; i <= buffer.length - pattern.length; i += 1) {
	        for (let j = 0; j < pattern.length; j += 1) {
	          if (buffer[i + j] !== pattern[j]) continue outer;
	        }
	        return i;
	      }
	      return -1;
	    }
	    function setVideoActive(cameraName) {
	      for (const name of ["front", "rear"]) {
	        const img = $(`${name}Video`);
	        const btn = $(`${name}VideoBtn`);
	        if (!img || !btn) continue;
	        const viewer = cameraViewers[name];
			        const active = name === cameraName;
			        if (active) {
			          if (viewer && !viewer.active) {
			            viewer.active = true;
			            viewer.token += 1;
			            pumpVideoFrames(name, viewer.token);
			          }
			          btn.textContent = "关闭";
			          btn.classList.add("active");
	        } else {
	          if (viewer) {
	            viewer.active = false;
	            viewer.token += 1;
	          }
	          clearVideoFrame(name);
	          btn.textContent = "打开";
	          btn.classList.remove("active");
	        }
	      }
	    }
	    function toggleVideo(cameraName) {
	      const viewer = cameraViewers[cameraName];
	      const isActive = !!viewer && viewer.active;
	      setVideoActive(isActive ? null : cameraName);
	    }
	    function fmtPose2d(pose, yawScale = 1.0) {
      if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) return "-";
      const yaw = Number(pose.display_yaw !== undefined ? pose.display_yaw : pose.yaw);
      const yawText = Number.isFinite(yaw) ? ` / ${fmtNumber(yaw * yawScale, yawScale === 1.0 ? 2 : 0)}${yawScale === 1.0 ? "rad" : "°"}` : "";
      return `x ${fmtNumber(Number(pose.x), 2)} / y ${fmtNumber(Number(pose.y), 2)}${yawText}`;
    }
    function planarError(a, b) {
      if (!a || !b) return null;
      const ax = Number(a.x);
      const ay = Number(a.y);
      const bx = Number(b.x);
      const by = Number(b.y);
      if (!Number.isFinite(ax) || !Number.isFinite(ay) || !Number.isFinite(bx) || !Number.isFinite(by)) return null;
      return Math.hypot(ax - bx, ay - by);
    }
    function navFeedbackPose(feedback) {
      if (!feedback) return null;
      return {
        x: feedback.pose_x,
        y: feedback.pose_y,
        yaw: feedback.pose_yaw
      };
    }
    function latestNavFeedbackPose(payload = state.latest) {
      if (!payload) return null;
      const active = payload.active_task || null;
      const waypoint = payload.active_waypoint && payload.active_waypoint.parsed ? payload.active_waypoint.parsed : null;
      const feedback = (waypoint && waypoint.nav_feedback) || (active && active.last_nav_feedback) || null;
      const pose = navFeedbackPose(feedback);
      if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) return null;
      let age = waypoint && waypoint.nav_feedback_age_s;
      if ((age === null || age === undefined) && active && active.last_nav_feedback_at) age = null;
      return {
        x: Number(pose.x),
        y: Number(pose.y),
        yaw: Number.isFinite(Number(pose.yaw)) ? Number(pose.yaw) : 0,
        age_s: Number.isFinite(Number(age)) ? Number(age) : null
      };
    }
    function poseUnavailableText(payload = state.latest) {
      if (!payload) return "等待状态";
      const poseAge = poseAgeSec(payload);
      if (payload.localization_ok === false) return "未定位，重定位后显示地图位姿";
      if (payload.localization_ok === true && payload.pose && !hasFreshPose(payload)) {
        const ageText = poseAge === null || poseAge === undefined ? "" : `，最后 ${fmtAge(poseAge)}前`;
        return `地图位姿过期${ageText}`;
      }
      if (payload.localization_ok === true && !payload.pose) return "定位已确认，但还未收到 /m20pro_tcp_bridge/map_pose";
      const odom = payload.odom && payload.odom.pose ? payload.odom.pose : null;
      if (odom && Number.isFinite(Number(odom.x)) && Math.abs(Number(odom.x)) > 100) {
        return "仅收到原厂 /ODOM，坐标未对齐当前地图";
      }
      return "等待地图位姿";
    }
    function renderLocalizationStatus(payload = state.latest) {
      const box = $("localizationStatus");
      if (!box) return;
      const rawStatus = payload && payload.localization_status ? payload.localization_status : null;
      box.className = "preflight-summary";
      if (!rawStatus) {
        box.classList.add("warn");
        box.textContent = "等待定位状态";
        return;
      }
      const status = {...rawStatus};
      const relocalization = payload && payload.relocalization_result ? payload.relocalization_result : null;
      if (!status.tcp_2101_result && relocalization && relocalization.raw) {
        const raw = String(relocalization.raw || "");
        status.tcp_2101_result = raw;
        status.tcp_2101_accepted = raw.startsWith("success");
        status.tcp_2101_failed = raw.startsWith("failed:");
        if (Number.isFinite(Number(payload && payload.node_time)) && Number.isFinite(Number(relocalization.last_update))) {
          status.tcp_2101_age_sec = Math.max(0, Number(payload.node_time) - Number(relocalization.last_update));
          status.tcp_2101_recent = status.tcp_2101_age_sec <= 300;
        }
      }
      if (!status.map_relocalization_required && payload && payload.task_readiness && payload.task_readiness.map_relocalization_required) {
        status.map_relocalization_required = payload.task_readiness.map_relocalization_required;
      }
      if (status.factory_localization_ok === undefined && payload && payload.factory_localization_ok !== undefined) {
        status.factory_localization_ok = payload.factory_localization_ok;
      }
      if (status.pose_fresh === undefined && payload && payload.pose_fresh !== undefined) {
        status.pose_fresh = payload.pose_fresh;
      }
      const confirmed = status.confirmed === true;
      const taskReady = status.task_ready === true;
      const mapLock = !!status.map_relocalization_required;
      const poseMismatch = status.pose_near_2101 === false;
      const finalSuccess = confirmed && !mapLock && !poseMismatch && status.pose_fresh !== false;
      box.classList.add(finalSuccess ? "ok" : "fail");
      box.innerHTML = "";
      const verdict = document.createElement("div");
      verdict.className = "localization-verdict";
      verdict.textContent = finalSuccess ? "重定位成功" : "重定位失败";
      box.appendChild(verdict);
      const message = document.createElement("div");
      message.className = "localization-message";
      message.textContent = finalSuccess
        ? (taskReady ? "定位已确认，任务页可启动" : (status.message || "定位已确认；任务页暂不可启动"))
        : (status.message || status.code || "未达到任务启动条件");
      box.appendChild(message);
      const poseAge = Number.isFinite(Number(status.pose_age_sec)) ? fmtAge(Number(status.pose_age_sec)) : "-";
      const poseError = Number.isFinite(Number(status.pose_error_m)) ? Number(status.pose_error_m) : null;
      const tcpAge = Number.isFinite(Number(status.tcp_2101_age_sec)) ? fmtAge(Number(status.tcp_2101_age_sec)) : null;
      const tcpResult = String(status.tcp_2101_result || "");
      let tcpText = "无回执";
      let tcpState = "fail";
      if (status.tcp_2101_accepted) {
        tcpText = `${status.tcp_2101_recent === false ? "旧回执" : "已收到回执"}${tcpAge ? " / " + tcpAge + "前" : ""}`;
        tcpState = finalSuccess ? "ok" : "warn";
      } else if (status.tcp_2101_failed) {
        tcpText = `失败回执${tcpAge ? " / " + tcpAge + "前" : ""}`;
      } else if (tcpResult) {
        tcpText = `未通过回执${tcpAge ? " / " + tcpAge + "前" : ""}`;
      }
      const factoryOk = status.factory_localization_ok === true || status.localization_ok === true;
      const poseText = status.pose_fresh
        ? `新鲜 / ${poseAge}前${poseError !== null ? ` / 距2101 ${poseError.toFixed(2)}m` : ""}`
        : (status.pose_ok ? `过期或未确认 / ${poseAge}前` : "缺失或无效");
      const poseState = status.pose_near_2101 === false ? "fail" : (status.pose_fresh ? "ok" : "warn");
      const rows = [
        ["2101回执", tcpText, tcpState],
        ["原厂定位", factoryOk ? "已确认" : "未确认", factoryOk ? "ok" : "fail"],
        ["地图位姿", poseText, poseState],
        ["固定地图", mapLock ? "重定位锁未清除" : "无重定位锁", mapLock ? "warn" : "ok"],
        ["任务页", taskReady ? "可启动" : "不可启动", taskReady ? "ok" : "fail"]
      ];
      const evidence = document.createElement("div");
      evidence.className = "localization-evidence";
      rows.forEach(([label, value, cls]) => {
        const row = document.createElement("div");
        row.className = "localization-row";
        const name = document.createElement("span");
        name.className = "localization-label";
        name.textContent = label;
        const stateText = document.createElement("span");
        stateText.className = `localization-value ${cls}`;
        stateText.textContent = value;
        row.appendChild(name);
        row.appendChild(stateText);
        evidence.appendChild(row);
      });
      box.appendChild(evidence);
    }
    function fmtAge(age) {
      if (age === null || age === undefined) return "-";
      if (age < 1.0) return "<1s";
      return `${age.toFixed(0)}s`;
    }
    function poseAgeSec(payload = state.latest) {
      if (!payload) return null;
      if (payload.pose_age_sec !== null && payload.pose_age_sec !== undefined) return Number(payload.pose_age_sec);
      const pose = payload.pose;
      if (!pose || payload.node_time === null || payload.node_time === undefined || pose.last_update === null || pose.last_update === undefined) return null;
      const age = Number(payload.node_time) - Number(pose.last_update);
      return Number.isFinite(age) ? Math.max(0, age) : null;
    }
    function hasFreshPose(payload = state.latest) {
      if (!payload || !payload.pose) return false;
      if (payload.pose_fresh === true) return true;
      if (payload.pose_fresh === false) return false;
      if (payload.localization_ok !== true) return false;
      const age = poseAgeSec(payload);
      return Number.isFinite(age) && age <= 3.0;
    }
    function freshPose(payload = state.latest) {
      return hasFreshPose(payload) ? payload.pose : null;
    }
    function yawDelta(from, to) {
      return Math.atan2(Math.sin(to - from), Math.cos(to - from));
    }
    function stableRobotDisplayPose(pose, activeTask = null) {
      if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) {
        state.robotDisplayPose = null;
        return null;
      }
      const raw = {
        x: Number(pose.x),
        y: Number(pose.y),
        yaw: normalizeYaw(Number.isFinite(Number(pose.display_yaw)) ? pose.display_yaw : pose.yaw)
      };
      if (activeTask && activeTask.status === "running") {
        state.robotDisplayPose = raw;
        return raw;
      }
      const prev = state.robotDisplayPose;
      if (!prev) {
        state.robotDisplayPose = raw;
        return raw;
      }
      const dx = raw.x - prev.x;
      const dy = raw.y - prev.y;
      const dist = Math.hypot(dx, dy);
      const dyaw = yawDelta(prev.yaw, raw.yaw);
      if (dist > 0.35 || Math.abs(dyaw) > 0.35) {
        state.robotDisplayPose = raw;
        return raw;
      }
      if (dist < 0.03 && Math.abs(dyaw) < 0.035) return prev;
      const alpha = 0.4;
      state.robotDisplayPose = {
        x: prev.x + dx * alpha,
        y: prev.y + dy * alpha,
        yaw: normalizeYaw(prev.yaw + dyaw * alpha)
      };
      return state.robotDisplayPose;
    }
    function mapRecordById(mapId) {
      const id = String(mapId || "");
      if (!id) return null;
      return state.maps.find(item => String(item.id || "") === id) || null;
    }
    function displayedMapFloor() {
      if (state.map && state.map.floor) return String(state.map.floor).trim();
      const record = mapRecordById(state.selectedMapId);
      return record && record.floor ? String(record.floor).trim() : "";
    }
    function currentRobotFloor(payload = state.latest) {
      return payload && payload.floor ? String(payload.floor).trim() : "";
    }
    function isViewingRobotFloor(payload = state.latest) {
      if (!state.selectedMapId) return true;
      const shownFloor = displayedMapFloor();
      const robotFloor = currentRobotFloor(payload);
      if (!shownFloor) return true;
      if (!robotFloor) return false;
      return shownFloor === robotFloor;
    }
    function selectedMapFloorMismatchText(payload = state.latest) {
      const shownFloor = displayedMapFloor();
      const robotFloor = currentRobotFloor(payload);
      if (!state.selectedMapId || !shownFloor) return "";
      if (!robotFloor) return `正在查看 ${shownFloor}，尚未收到机器狗真实楼层`;
      if (shownFloor !== robotFloor) return `正在查看 ${shownFloor}，机器狗实际在 ${robotFloor}`;
      return "";
    }
    function updateFloorDisplay(payload = state.latest) {
      const shownFloor = displayedMapFloor() || (state.selectedMapId ? "-" : "实时");
      const robotFloor = currentRobotFloor(payload) || "-";
      const mismatch = selectedMapFloorMismatchText(payload);
      const badge = $("mapFloorBadge");
      if (badge) {
        badge.textContent = `查看 ${shownFloor} / 真实 ${robotFloor}`;
        badge.className = `floor-badge ${mismatch ? "warn" : "ok"}`;
      }
      const overlay = $("floorOverlay");
      if (overlay) {
        overlay.textContent = mismatch || `当前显示 ${shownFloor}`;
        overlay.className = `floor-overlay ${mismatch ? "warn" : ""}`;
      }
    }
    function localizationConfirmedForDisplay(payload = state.latest) {
      const status = payload && payload.localization_status ? payload.localization_status : null;
      return !!(
        status
        && status.confirmed === true
        && status.pose_fresh !== false
        && status.pose_near_2101 !== false
        && !status.map_relocalization_required
      );
    }
    function markBlockedReason(payload = state.latest) {
      if (!state.map) return "还没有地图，等地图加载后再标点";
      if (!state.selectedMapId) return "先选择固定地图；实时 /map 只用于临时观察，不能保存任务点";
      const shownFloor = displayedMapFloor();
      const inputFloor = $("markFloor") ? $("markFloor").value.trim() : "";
      if (shownFloor && inputFloor && shownFloor !== inputFloor) {
        return `当前显示 ${shownFloor} 地图，点位楼层填的是 ${inputFloor}；请先确认楼层`;
      }
      return "";
    }
    function updateMarkControls(payload = state.latest) {
      const saveBtn = $("saveMarkBtn");
      const usePoseBtn = $("useRobotPoseBtn");
      const reason = markBlockedReason(payload);
      if (saveBtn) {
        saveBtn.disabled = !!reason;
        saveBtn.title = reason || "保存当前固定地图上的点位";
      }
      if (usePoseBtn) {
        const mismatch = selectedMapFloorMismatchText(payload);
        const selectedMapStatus = (payload && payload.selected_map_status) || state.selectedMapStatus;
        let poseReason = "";
        if (mismatch) poseReason = `${mismatch}，不能把实时位姿保存到这张地图`;
        else if (selectedMapStatus && selectedMapStatus.ready === false) {
          poseReason = selectedMapStatus.message || "网页选择地图与 Nav2 当前加载地图不一致，请先切换到正确地图并重定位";
        } else if (!payload || !hasFreshPose(payload)) {
          poseReason = "先完成重定位，看到定位页显示重定位成功并收到实时位姿后再取当前位姿";
        }
        usePoseBtn.disabled = !!poseReason || !state.selectedMapId;
        usePoseBtn.title = poseReason || (state.selectedMapId ? "使用当前机器人位姿填入点位" : "先选择固定地图");
      }
    }
    function formatUsageMode(value) {
      if (value === null || value === undefined || value === "") return null;
      const map = {
        0: "常规",
        1: "导航",
        2: "辅助"
      };
      const key = Number(value);
      const label = Number.isFinite(key) && Object.prototype.hasOwnProperty.call(map, key) ? map[key] : String(value);
      return `使用模式 ${label}`;
    }
    function formatOoa(value) {
      if (value === null || value === undefined || value === "") return null;
      const map = {
        0: "未启动",
        1: "空闲中",
        2: "未触发避障",
        3: "主动避障中"
      };
      const key = Number(value);
      const label = Number.isFinite(key) && Object.prototype.hasOwnProperty.call(map, key) ? map[key] : String(value);
      return `辅助避障 ${label}`;
    }
    function setLog(id, payload) {
      $(id).textContent = typeof payload === "string" ? payload : JSON.stringify(payload, null, 2);
    }
    function sleepMs(ms) {
      return new Promise(resolve => setTimeout(resolve, ms));
    }
    function preflightStatusText(result) {
      if (!result) return "尚未自检";
      const ageText = result.age_sec === null || result.age_sec === undefined ? "" : ` / ${fmtAge(result.age_sec)}前`;
      if (result.running) return `${result.summary || "基础自检后台执行中，请稍候"}${ageText}`;
      if (result.summary) return `${result.summary}${ageText}`;
      if (result.ok && result.navigation_ready === false) return `最近一次基础自检通过；导航待重定位${ageText}`;
      if (result.ok) return `最近一次基础自检通过${ageText}`;
      return `最近一次基础自检未通过${ageText}`;
    }
    function renderPreflight(result) {
      state.preflight = result || null;
      const summaries = [$("preflightSummary"), $("taskPreflightSummary")];
      for (const box of summaries) {
        if (!box) continue;
        box.className = "preflight-summary";
        if (result) {
          const cls = result.running ? "warn" : (result.ok ? "ok" : "fail");
          box.classList.add(cls);
        }
        box.textContent = preflightStatusText(result);
      }
      const itemsBox = $("preflightItems");
      if (itemsBox) {
        itemsBox.innerHTML = "";
        const items = result && result.items ? result.items : [];
        if (!items.length) {
          itemsBox.innerHTML = `<div class="small">尚未自检。</div>`;
        } else {
          for (const item of items) {
            const row = document.createElement("div");
            row.className = "check-row";
            const statusClass = item.status === "ok" ? "ok" : (item.status === "warn" ? "warn" : (item.status === "info" ? "ok" : "fail"));
            const statusText = item.status === "ok" ? "通过" : (item.status === "warn" ? "提醒" : (item.status === "info" ? "信息" : "失败"));
            row.innerHTML = `
              <div class="check-status ${statusClass}">${statusText}</div>
              <div><strong>${item.label || item.key}</strong><div class="small">${item.message || ""}</div></div>
            `;
            itemsBox.appendChild(row);
          }
        }
      }
      if ($("preflightRaw")) $("preflightRaw").textContent = result ? JSON.stringify(result, null, 2) : "等待自检";
    }
    function renderTaskReadiness(readiness) {
      state.taskReadiness = readiness || null;
      const box = $("taskReadinessSummary");
      if (!box) return;
      box.className = "preflight-summary";
      if (!readiness) {
        box.classList.add("warn");
        box.textContent = "等待任务链路状态";
        return;
      }
      box.classList.add(readiness.ready ? "ok" : "warn");
      const age = readiness.pose_age_sec === null || readiness.pose_age_sec === undefined
        ? ""
        : ` / 位姿 ${fmtNumber(Number(readiness.pose_age_sec), 1)}s前`;
      const firstDistance = Number(readiness.first_waypoint_distance_m);
      const distance = Number.isFinite(firstDistance) ? ` / 首点 ${fmtNumber(firstDistance, 2)}m` : "";
      const warnDistance = Number(readiness.first_waypoint_distance_warn_m);
      if (readiness.ready && Number.isFinite(firstDistance) && Number.isFinite(warnDistance) && warnDistance > 0 && firstDistance > warnDistance) {
        box.classList.remove("ok");
        box.classList.add("warn");
      }
      const displayMessage = readiness.code === "pose_invalid_or_stale" && localizationConfirmedForDisplay()
        ? "定位已确认，等待地图位姿刷新后再开始任务"
        : (readiness.message || (readiness.ready ? "任务链路可用" : "任务链路未就绪"));
      box.textContent = `${displayMessage}${age}${distance}`;
      renderTaskNextStep();
    }
    function renderTaskNextStep() {
      const box = $("taskNextStepSummary");
      if (!box) return;
      const readiness = state.taskReadiness || {};
      const code = readiness.code || "";
      const currentMapTasks = state.tasks.filter(task => taskBelongsToSelectedMap(task));
      const currentMapPoints = state.annotations.length;
      const selectedMapStatus = state.selectedMapStatus || {};
      const perceptionStatus = state.latest && state.latest.perception_status ? state.latest.perception_status : {};
      box.className = "preflight-summary";
      if (readiness.ready === true) {
        box.classList.add("ok");
        box.textContent = "下一步：执行前确认首点、顺序和开跑前验收命令，再人工点击开始任务";
        return;
      }
      box.classList.add("warn");
      if (code === "battery_low") {
        box.textContent = "下一步：先充电；电量恢复后再重定位和建任务";
      } else if (perceptionStatus.ready === false && [
        "factory_lidar_points_publisher_missing",
        "lidar_relay_no_samples",
        "lidar_relay_output_unavailable",
        "scan_unavailable",
      ].includes(perceptionStatus.code || "")) {
        box.textContent = `下一步：${perceptionStatus.message || "先恢复点云 relay 和 /scan 感知链路"}`;
      } else if (selectedMapStatus.ready === false) {
        box.textContent = `下一步：${selectedMapStatus.message || "网页选择地图与 Nav2 当前加载地图不一致，请先切换到正确地图并重定位"}`;
      } else if (code === "localization_not_confirmed") {
        box.textContent = "下一步：到定位页完成重定位，必须看到重定位成功";
      } else if (code === "pose_invalid_or_stale" && localizationConfirmedForDisplay()) {
        box.textContent = "下一步：定位已确认，等待地图位姿刷新；蓝色箭头或激光明显不对时再重新定位";
      } else if (code === "pose_invalid_or_stale") {
        box.textContent = "下一步：到定位页完成重定位，必须看到重定位成功";
      } else if (!currentMapPoints) {
        box.textContent = "下一步：当前地图没有任务点；重定位成功后，到标点页在当前地图保存点位";
      } else if (!currentMapTasks.length) {
        box.textContent = "下一步：当前地图还没有任务；勾选当前地图点位并生成任务";
      } else {
        box.textContent = `下一步：${readiness.message || "按任务页执行条件处理当前阻塞项"}`;
      }
    }
    function taskStartButtonLabel(readiness) {
      if (!readiness) return "检查中";
      if (readiness.ready) return "开始执行";
      const code = readiness.code || "";
      if (code === "localization_not_confirmed") return "先重定位";
      if (code === "pose_invalid_or_stale") return localizationConfirmedForDisplay() ? "等位姿" : "先重定位";
      if (code === "selected_map_mismatch" || code === "map_metadata_mismatch" || code === "map_unavailable") return "检查地图";
      if (code === "wrong_floor" || code === "floor_unknown") return "检查楼层";
      if (code === "target_out_of_map" || code === "current_pose_out_of_map" || code === "waypoint_out_of_map") return "点位越界";
      if (code === "waypoint_on_occupied_cell" || code === "waypoint_on_unknown_cell") return "检查点位";
      if (code === "first_waypoint_too_far") return "首点过远";
      if (code === "navigation_not_ready") return "等导航";
      if (code === "battery_missing" || code === "battery_stale") return "等电池";
      if (code === "battery_low") return "先充电";
      if (code === "perception_scan_unavailable") return "等scan";
      if (code === "perception_lidar_unavailable") return "等点云";
      if (code === "no_waypoint" || code === "missing_waypoint" || code === "task_invalid") return "任务无效";
      return "不可执行";
    }
    function normalizedTaskStatus(status) {
      return String(status || "ready");
    }
    function taskStatusAllowsStart(status) {
      const normalized = normalizedTaskStatus(status);
      return normalized === "ready" || normalized === "stopped" || normalized === "completed" || normalized === "error";
    }
    function taskStartLabelForStatus(status, readiness) {
      const normalized = normalizedTaskStatus(status);
      if (normalized === "error") return "从头重试";
      if (normalized === "invalid") return "任务无效";
      if (normalized === "running") return "执行中";
      if (!taskStatusAllowsStart(normalized)) return "不可执行";
      if (!readiness || !readiness.ready) return taskStartButtonLabel(readiness);
      if (normalized === "completed") return "重新执行";
      if (normalized === "stopped") return "从头执行";
      return "开始执行";
    }
    function taskBelongsToSelectedMap(task) {
      const selected = String(state.selectedMapId || "");
      if (!selected || !task) return false;
      if (String(task.map_id || "") === selected) return true;
      const mapIds = Array.isArray(task.map_ids) ? task.map_ids.map(item => String(item || "")) : [];
      return mapIds.includes(selected);
    }
    function taskMapMismatchText(task) {
      if (taskBelongsToSelectedMap(task)) return "";
      const selected = state.maps.find(item => String(item.id || "") === String(state.selectedMapId || ""));
      const selectedName = selected ? (selected.name || selected.id) : (state.selectedMapId || "-");
      return `该任务属于 ${task.map_id || "-"}，当前地图是 ${selectedName}；请在当前地图重新标点生成任务`;
    }
    function taskStatusBlockText(status) {
      const normalized = normalizedTaskStatus(status);
      if (normalized === "error") return "";
      if (normalized === "invalid") return "任务点位已失效，请重新生成任务";
      if (normalized === "running") return "任务正在执行中";
      if (!taskStatusAllowsStart(normalized)) return `任务状态 ${normalized} 不允许启动`;
      return "";
    }
    function taskDisplayStatus(taskStatus, readiness, mapMismatchText) {
      if (mapMismatchText) return "旧地图";
      if (readiness && readiness.ready === true && readiness.multi_floor) return "跨楼层";
      if (taskStatus === "running") return "执行中";
      if (taskStatus === "invalid") return "无效";
      if (readiness && readiness.ready === true) return "可执行";
      return "未就绪";
    }
    function taskWaypointText(point, index) {
      if (!point) return `${index + 1}. -`;
      const pose = point.pose || {};
      const name = point.missing ? `${point.id || "缺失点"}(缺失)` : (point.label || point.id || `点${index + 1}`);
      const xy = Number.isFinite(Number(pose.x)) && Number.isFinite(Number(pose.y))
        ? ` x${fmtNumber(Number(pose.x), 2)} y${fmtNumber(Number(pose.y), 2)}`
        : "";
      const yaw = Number.isFinite(Number(pose.yaw)) ? ` yaw${fmtNumber(Number(pose.yaw), 2)}` : "";
      const dwell = Number.isFinite(Number(point.dwell_s)) ? ` 停${fmtNumber(Number(point.dwell_s), 1)}s` : "";
      return `${index + 1}.${point.floor || "-"} ${name}${xy}${yaw}${dwell}`;
    }
    function taskStartRequest(task) {
      const waypoints = Array.isArray(task.waypoints) ? task.waypoints : [];
      const first = waypoints.length ? waypoints[0] : null;
      return {
        task_id: task.id,
        expected_annotation_ids: Array.isArray(task.annotation_ids) ? task.annotation_ids.slice() : [],
        expected_first_annotation_id: first ? first.id : ((task.annotation_ids || [])[0] || ""),
        expected_first_pose: first && first.pose ? first.pose : null,
        expected_map_id: task.map_id || "",
        expected_task_updated_at: task.updated_at || task.created_at || "",
      };
    }
    function taskFirstDistanceText(readiness) {
      const value = readiness && Number(readiness.first_waypoint_distance_m);
      if (!Number.isFinite(value)) return "";
      const warn = Number(readiness.first_waypoint_distance_warn_m);
      const max = Number(readiness.first_waypoint_distance_max_m);
      const flags = [];
      if (Number.isFinite(max) && max > 0 && value > max) flags.push("超过上限");
      else if (Number.isFinite(warn) && warn > 0 && value > warn) flags.push("偏远");
      return `当前位置到首点 ${fmtNumber(value, 2)}m${flags.length ? `（${flags.join("，")}）` : ""}`;
    }
    function shellQuote(value) {
      return `'${String(value || "").replace(/'/g, `'\"'\"'`)}'`;
    }
    function taskWatcherLabel(task) {
      const taskId = String(task && task.id || "");
      const suffix = taskId ? taskId.slice(-8) : "";
      return `field_task${suffix ? `_${suffix}` : ""}`;
    }
    function taskWatcherCommand(task) {
      return `./scripts/104_watch_frontend_task.sh 180 ${shellQuote(taskWatcherLabel(task))}`;
    }
    function taskReadyCheckCommand(task) {
      return `./scripts/104_frontend_task_ready_check.py --task-id ${shellQuote(task && task.id || "")}`;
    }
    async function copyTextToClipboard(value) {
      const text = String(value || "");
      if (!text) return false;
      if (navigator.clipboard && navigator.clipboard.writeText) {
        await navigator.clipboard.writeText(text);
        return true;
      }
      const area = document.createElement("textarea");
      area.value = text;
      area.setAttribute("readonly", "readonly");
      area.style.position = "fixed";
      area.style.left = "-9999px";
      document.body.appendChild(area);
      area.select();
      try {
        return document.execCommand("copy");
      } finally {
        document.body.removeChild(area);
      }
    }
    function compactTaskForSnapshot(task) {
      if (!task) return null;
      const waypoints = Array.isArray(task.waypoints) ? task.waypoints : [];
      return {
        id: task.id || null,
        name: task.name || null,
        status: task.status || null,
        map_id: task.map_id || null,
        readiness: task.readiness || null,
        waypoint_count: waypoints.length,
        first_waypoint: waypoints.length ? {
          id: waypoints[0].id || null,
          label: waypoints[0].label || null,
          floor: waypoints[0].floor || null,
          pose: waypoints[0].pose || null,
          dwell_s: waypoints[0].dwell_s,
        } : null,
        order: waypoints.map((point, index) => taskWaypointText(point, index)),
        last_result: task.last_result || null,
      };
    }
    function taskExecutionEvidence(activeTask, activeWaypoint) {
      const source = activeWaypoint || {};
      const navFeedback = source.nav_feedback || (activeTask && activeTask.last_nav_feedback) || null;
      const navGoalMatch = source.nav_goal_match || (activeTask && activeTask.last_nav_goal_match) || null;
      return {
        task_id: activeTask ? activeTask.task_id : null,
        task_name: activeTask ? activeTask.task_name : null,
        phase: source.phase || (activeTask && activeTask.phase) || null,
        waypoint_id: source.waypoint && source.waypoint.id ? source.waypoint.id : (activeTask && activeTask.last_goal_annotation_id) || null,
        waypoint_label: source.waypoint && source.waypoint.label ? source.waypoint.label : (activeTask && activeTask.last_goal_label) || null,
        goal_attempt_id: source.goal_attempt_id || (activeTask && activeTask.last_goal_attempt_id) || null,
        floor_goal_published_at: source.last_floor_goal_published_at || (activeTask && activeTask.last_floor_goal_published_at) || null,
        floor_goal_source_floor: source.last_floor_goal_source_floor || (activeTask && activeTask.last_floor_goal_source_floor) || null,
        floor_goal_target_floor: source.last_floor_goal_target_floor || (activeTask && activeTask.last_floor_goal_target_floor) || null,
        floor_goal_cross_floor: source.last_floor_goal_cross_floor !== undefined
          ? source.last_floor_goal_cross_floor
          : (activeTask && activeTask.last_floor_goal_cross_floor) || false,
        floor_goal_publish_count: source.floor_goal_publish_count !== undefined
          ? source.floor_goal_publish_count
          : (activeTask && activeTask.floor_goal_publish_count) || null,
        transition_nav_status: source.last_transition_nav_status || (activeTask && activeTask.last_transition_nav_status) || null,
        transition_nav_label: source.last_transition_nav_label || (activeTask && activeTask.last_transition_nav_label) || null,
        transition_nav_payload: source.last_transition_nav_payload || (activeTask && activeTask.last_transition_nav_payload) || null,
        nav_goal_status: source.nav_goal_status || (activeTask && activeTask.last_nav_goal_status) || null,
        nav_goal_seq: navFeedback && navFeedback.goal_seq !== undefined
          ? navFeedback.goal_seq
          : (navGoalMatch && navGoalMatch.nav_goal_seq !== undefined ? navGoalMatch.nav_goal_seq : null),
        nav_goal_matches: navGoalMatch && navGoalMatch.matches !== undefined ? navGoalMatch.matches : null,
        nav_feedback_age_s: source.nav_feedback_age_s !== undefined ? source.nav_feedback_age_s : null,
        nav_distance_remaining_m: navFeedback && navFeedback.distance_remaining !== undefined ? navFeedback.distance_remaining : null,
        plan_goal_verified: source.plan_goal_verified !== undefined ? source.plan_goal_verified : (activeTask && activeTask.plan_goal_verified) || null,
        path_goal_error_m: source.path_goal_error_m !== undefined ? source.path_goal_error_m : null,
        status_message: source.status_message || (activeTask && activeTask.status_message) || null,
      };
    }
    function buildFieldSnapshot() {
      const latest = state.latest || {};
      const lidar = latest.lidar_points || {};
      const lidarRelay = latest.lidar_relay_status || {};
      const scan = latest.scan || {};
      const activeWaypoint = latest.active_waypoint && latest.active_waypoint.parsed ? latest.active_waypoint.parsed : null;
      const currentMapTasks = state.tasks.filter(task => taskBelongsToSelectedMap(task));
      const recommendedTask = currentMapTasks.find(task => task && task.readiness && task.readiness.code === "ready")
        || currentMapTasks[0]
        || null;
      return {
        captured_at: new Date().toISOString(),
        frontend: {
          selected_map_id: state.selectedMapId,
          map_mode_label: state.mapModeLabel,
          status_text: $("statusText") ? $("statusText").textContent : null,
        },
        robot: {
          floor: latest.floor,
          localization_ok: latest.localization_ok,
          pose_fresh: latest.pose_fresh,
          pose_age_sec: latest.pose_age_sec,
          pose: latest.pose || null,
          navigation_status: latest.navigation_status || null,
          battery: latest.battery && latest.battery.primary ? latest.battery.primary : null,
        },
        perception: {
          status: latest.perception_status || null,
          scan_finite_ranges: scan.finite_ranges,
          scan_age_sec: latest.node_time && scan.last_update ? Math.max(0, Number(latest.node_time) - Number(scan.last_update)) : null,
          lidar_points: (Number(lidar.width || 0) || 0) * Math.max(1, Number(lidar.height || 1) || 1),
          lidar_source: lidar.source || null,
          lidar_relay: {
            output_width: lidarRelay.output_width,
            output_height: lidarRelay.output_height,
            output_stride: lidarRelay.output_stride,
            input_rate_hz: lidarRelay.input_rate_hz,
            publish_rate_hz: lidarRelay.publish_rate_hz,
            skip_ratio: lidarRelay.skip_ratio,
            downsample_method: lidarRelay.downsample_method,
          },
        },
        task_readiness: latest.task_readiness || state.taskReadiness || null,
        active_task: latest.active_task || null,
        active_waypoint: activeWaypoint,
        task_execution_evidence: taskExecutionEvidence(latest.active_task || null, activeWaypoint),
        last_task_result: latest.last_task_result || null,
        recommended_task: compactTaskForSnapshot(recommendedTask),
        task_pose_tracker_text: $("taskPoseTracker") ? $("taskPoseTracker").textContent : null,
        active_task_summary_text: $("activeTaskSummary") ? $("activeTaskSummary").textContent : null,
      };
    }
    async function copyFieldSnapshot() {
      const snapshot = buildFieldSnapshot();
      return copyTextToClipboard(JSON.stringify(snapshot, null, 2));
    }
    function taskStartConfirmText(task, readinessText) {
      const waypoints = Array.isArray(task.waypoints) ? task.waypoints : [];
      const first = waypoints.length ? waypoints[0] : null;
      const sequence = waypoints.length
        ? waypoints.map((point, index) => taskWaypointText(point, index)).join(" → ")
        : "无点位";
      const firstDistanceText = taskFirstDistanceText(task.readiness || {});
      return [
        `确认开始任务：${task.name || task.id}`,
        `首点：${first ? taskWaypointText(first, 0) : "-"}`,
        firstDistanceText ? `距离：${firstDistanceText}` : null,
        `顺序：${sequence}`,
        `执行条件：${readinessText || "-"}`,
        `先验收：${taskReadyCheckCommand(task)}`,
        `再开记录：${taskWatcherCommand(task)}`,
        "确认后机器狗会立即向首点导航。"
      ].filter(Boolean).join("\n");
    }
    function taskLastResultText(task) {
      const result = task && task.last_result ? task.last_result : null;
      if (!result) {
        const timeline = Array.isArray(task && task.last_timeline) ? task.last_timeline : [];
        const last = timeline.length ? timeline[timeline.length - 1] : null;
        return last && last.message ? `上次事件：${last.message}` : "";
      }
      const parts = [];
      if (result.status) parts.push(`上次${result.status}`);
      if (result.message) parts.push(result.message);
      if (result.reason) parts.push(`原因 ${result.reason}`);
      const wp = result.waypoint || {};
      if (wp.label) parts.push(`点位 ${wp.label}`);
      if (Number.isFinite(Number(result.last_distance_m))) {
        parts.push(`距离 ${fmtNumber(Number(result.last_distance_m), 2)}m`);
      } else if (Number.isFinite(Number(result.distance_m))) {
        parts.push(`距离 ${fmtNumber(Number(result.distance_m), 2)}m`);
      }
      if (Number.isFinite(Number(result.path_goal_error_m))) parts.push(`路径差 ${fmtNumber(Number(result.path_goal_error_m), 2)}m`);
      if (result.last_nav_goal_status) parts.push(`Nav2 ${result.last_nav_goal_status}`);
      if (result.plan_goal_verified === true) parts.push("路径已校验");
      if (result.saved_at) parts.push(result.saved_at);
      return parts.length ? parts.join(" / ") : "";
    }
    function renderActiveTaskSummary(activeTask, waypoint) {
      const box = $("activeTaskSummary");
      if (!box) return;
      box.className = "preflight-summary";
      if (!activeTask && !waypoint) {
        box.textContent = "无任务";
        return;
      }
      const source = waypoint || {};
      const wp = source.waypoint || {};
      const parts = [];
      if (activeTask && activeTask.task_name) parts.push(activeTask.task_name);
      if (wp.label) parts.push(`点位 ${wp.label}`);
      if (Number.isFinite(Number(source.index))) parts.push(`序号 ${Number(source.index) + 1}`);
      if (source.phase) parts.push(source.phase === "dwelling" ? "停留中" : "导航中");
      if (source.nav_goal_status) parts.push(`Nav2 ${source.nav_goal_status}`);
      if (Number.isFinite(Number(source.distance_m))) parts.push(`距离 ${fmtNumber(Number(source.distance_m), 2)}m`);
      const navFeedback = source.nav_feedback || (activeTask && activeTask.last_nav_feedback) || null;
      const navGoalMatch = source.nav_goal_match || (activeTask && activeTask.last_nav_goal_match) || null;
      if (navFeedback) {
        if (navFeedback.goal_seq !== undefined && navFeedback.goal_seq !== null) {
          parts.push(`Nav2序号 ${navFeedback.goal_seq}`);
        }
        if (Number.isFinite(Number(source.nav_feedback_age_s))) {
          parts.push(`Nav2反馈 ${fmtAge(Number(source.nav_feedback_age_s))}前`);
        }
        if (Number.isFinite(Number(navFeedback.distance_remaining))) {
          parts.push(`Nav2剩余 ${fmtNumber(Number(navFeedback.distance_remaining), 2)}m`);
        }
        if (Number.isFinite(Number(navFeedback.navigation_time))) {
          parts.push(`Nav2 ${fmtNumber(Number(navFeedback.navigation_time), 0)}s`);
        }
        if (Number.isFinite(Number(navFeedback.recoveries)) && Number(navFeedback.recoveries) > 0) {
          parts.push(`恢复 ${Number(navFeedback.recoveries)}次`);
        }
      }
      if (source.goal_attempt_id || (activeTask && activeTask.last_goal_attempt_id)) {
        parts.push(`尝试 ${(source.goal_attempt_id || activeTask.last_goal_attempt_id)}`);
      }
      const floorGoalPublishedAt = source.last_floor_goal_published_at || (activeTask && activeTask.last_floor_goal_published_at);
      if (floorGoalPublishedAt) parts.push(`floor_goal已发 ${floorGoalPublishedAt}`);
      const floorGoalCross = source.last_floor_goal_cross_floor !== undefined
        ? source.last_floor_goal_cross_floor
        : (activeTask && activeTask.last_floor_goal_cross_floor);
      const floorGoalSource = source.last_floor_goal_source_floor || (activeTask && activeTask.last_floor_goal_source_floor);
      const floorGoalTarget = source.last_floor_goal_target_floor || (activeTask && activeTask.last_floor_goal_target_floor);
      if (floorGoalCross && floorGoalSource && floorGoalTarget) {
        parts.push(`跨楼层 ${floorGoalSource}->${floorGoalTarget}`);
      }
      const floorGoalPublishes = source.floor_goal_publish_count !== undefined
        ? source.floor_goal_publish_count
        : (activeTask && activeTask.floor_goal_publish_count);
      if (Number.isFinite(Number(floorGoalPublishes))) parts.push(`/floor_goal ${Number(floorGoalPublishes)}次`);
      const transitionStatus = source.last_transition_nav_status || (activeTask && activeTask.last_transition_nav_status);
      const transitionLabel = source.last_transition_nav_label || (activeTask && activeTask.last_transition_nav_label);
      const transitionPayload = source.last_transition_nav_payload || (activeTask && activeTask.last_transition_nav_payload) || null;
      if (transitionLabel) parts.push(`楼梯阶段 ${transitionLabel}`);
      if (transitionPayload && Number.isFinite(Number(transitionPayload.distance_remaining))) {
        parts.push(`楼梯剩余 ${fmtNumber(Number(transitionPayload.distance_remaining), 2)}m`);
      } else if (transitionStatus) {
        parts.push(`楼梯状态 ${transitionStatus}`);
      }
      if (navGoalMatch) {
        if (navGoalMatch.nav_goal_seq !== undefined && navGoalMatch.nav_goal_seq !== null && !(navFeedback && navFeedback.goal_seq !== undefined && navFeedback.goal_seq !== null)) {
          parts.push(`Nav2序号 ${navGoalMatch.nav_goal_seq}`);
        }
        if (navGoalMatch.matches === false) {
          parts.push(`目标不匹配 ${navGoalMatch.reason || ""}`.trim());
        }
      }
      const ignoredMatch = source.last_ignored_nav_goal_match || (activeTask && activeTask.last_ignored_nav_goal_match) || null;
      if (ignoredMatch && ignoredMatch.reason) parts.push(`已忽略旧反馈 ${ignoredMatch.reason}`);
      const robotPose = source.robot_pose || (activeTask && activeTask.last_robot_pose) || null;
      const statePose = source.state_pose || null;
      const goalPose = source.goal_pose || (wp && wp.pose) || null;
      if (robotPose && Number.isFinite(Number(robotPose.x)) && Number.isFinite(Number(robotPose.y))) {
        parts.push(`狗 x${fmtNumber(Number(robotPose.x), 2)} y${fmtNumber(Number(robotPose.y), 2)}`);
      }
      if (statePose && Number.isFinite(Number(statePose.x)) && Number.isFinite(Number(statePose.y)) && (!robotPose || planarError(robotPose, statePose) > 0.05)) {
        parts.push(`地图位姿 x${fmtNumber(Number(statePose.x), 2)} y${fmtNumber(Number(statePose.y), 2)}`);
      }
      if (goalPose && Number.isFinite(Number(goalPose.x)) && Number.isFinite(Number(goalPose.y))) {
        parts.push(`目标 x${fmtNumber(Number(goalPose.x), 2)} y${fmtNumber(Number(goalPose.y), 2)}`);
      }
      const navPose = navFeedbackPose(navFeedback);
      const robotGoalError = planarError(robotPose, goalPose);
      const navGoalError = planarError(navPose, goalPose);
      const robotNavError = planarError(robotPose, navPose);
      const stateNavError = planarError(statePose, navPose);
      const pathGoalError = Number(source.path_goal_error_m);
      if (Number.isFinite(robotGoalError)) parts.push(`狗差 ${fmtNumber(robotGoalError, 2)}m`);
      if (Number.isFinite(navGoalError)) parts.push(`Nav2差 ${fmtNumber(navGoalError, 2)}m`);
      if (Number.isFinite(robotNavError)) parts.push(`位姿差 ${fmtNumber(robotNavError, 2)}m`);
      if (Number.isFinite(stateNavError)) parts.push(`反馈差 ${fmtNumber(stateNavError, 2)}m`);
      if (Number.isFinite(pathGoalError)) parts.push(`路径差 ${fmtNumber(pathGoalError, 2)}m`);
      if (source.plan_goal_verified === true || (activeTask && activeTask.plan_goal_verified === true)) {
        parts.push("路径已校验");
      }
      const sentPathVersion = source.goal_sent_path_version !== undefined ? source.goal_sent_path_version : (activeTask && activeTask.goal_sent_path_version);
      const planPathVersion = source.plan_path_version !== undefined ? source.plan_path_version : (activeTask && activeTask.plan_path_version);
      if (sentPathVersion !== undefined && sentPathVersion !== null) parts.push(`下发路径版 ${sentPathVersion}`);
      if (planPathVersion !== undefined && planPathVersion !== null) parts.push(`校验路径版 ${planPathVersion}`);
      if (Number.isFinite(Number(source.remaining_dwell_s)) && Number(source.remaining_dwell_s) > 0) {
        parts.push(`剩余停留 ${fmtNumber(Number(source.remaining_dwell_s), 1)}s`);
      }
      if (Number.isFinite(Number(source.elapsed_s))) parts.push(`本点 ${fmtNumber(Number(source.elapsed_s), 0)}s`);
      if (Number.isFinite(Number(source.goal_send_count))) parts.push(`目标下发 ${Number(source.goal_send_count)}次`);
      const stallAge = source.stall_age_s !== undefined ? source.stall_age_s : (activeTask && activeTask.stall_age_s);
      if (Number.isFinite(Number(stallAge)) && Number(stallAge) > 0) {
        parts.push(`低进展 ${fmtNumber(Number(stallAge), 0)}s`);
      }
      const runtimeGuard = source.runtime_guard || (activeTask && activeTask.runtime_guard) || null;
      if (runtimeGuard && runtimeGuard.code) {
        parts.push(`链路守护 ${runtimeGuard.code}`);
      }
      const lastProgressAt = source.last_progress_at || (activeTask && activeTask.last_progress_at);
      if (lastProgressAt) parts.push(`最近进展 ${lastProgressAt}`);
      const msg = source.status_message || (activeTask && activeTask.status_message) || "";
      if (msg) parts.push(msg);
      const timeline = activeTask && Array.isArray(activeTask.timeline) ? activeTask.timeline : [];
      const lastEvent = timeline.length ? timeline[timeline.length - 1] : null;
      if (lastEvent && lastEvent.message) parts.push(`最近事件 ${lastEvent.message}`);
      box.classList.add((activeTask && activeTask.last_error) ? "fail" : "ok");
      box.textContent = parts.length ? parts.join(" / ") : "任务执行中";
    }
    function updateTaskControlButtons(payload = state.latest) {
      const stopBtn = $("stopTaskBtn");
      const resetBtn = $("resetTaskSessionBtn");
      const hasActiveTask = !!(payload && (payload.active_task || payload.active_waypoint));
      if (stopBtn) {
        stopBtn.disabled = !hasActiveTask;
        stopBtn.title = hasActiveTask
          ? "停止当前前端任务"
          : "当前没有前端任务在执行";
      }
      if (resetBtn) {
        resetBtn.disabled = false;
        resetBtn.title = "显式复位导航会话；会停止前端任务、清理导航会话并清代价地图";
      }
    }
    function renderPoseTracker(targetId, payload = state.latest) {
      const box = $(targetId);
      if (!box) return;
      if (!payload) {
        box.textContent = "等待位姿";
        return;
      }
      const activeTask = payload.active_task || null;
      const waypoint = payload.active_waypoint && payload.active_waypoint.parsed ? payload.active_waypoint.parsed : null;
      const wp = waypoint && waypoint.waypoint ? waypoint.waypoint : null;
      const goalPose = (waypoint && waypoint.goal_pose) || (wp && wp.pose) || (activeTask && activeTask.last_goal_pose) || null;
      const navFeedback = (waypoint && waypoint.nav_feedback) || (activeTask && activeTask.last_nav_feedback) || null;
      const navPose = navFeedbackPose(navFeedback);
      const robotPose = (waypoint && waypoint.robot_pose) || (activeTask && activeTask.last_robot_pose) || payload.pose || null;
      const currentFresh = hasFreshPose(payload);
      const poseAge = poseAgeSec(payload);
      const rows = [];
      const addRow = (label, value, cls = "") => {
        rows.push(`<div class="pose-track-row ${cls}"><strong>${label}</strong><span>${value}</span></div>`);
      };
      const poseCls = payload.localization_ok === true && currentFresh ? "ok" : (payload.localization_ok === false ? "fail" : "warn");
      if (payload.pose) {
        const ageText = poseAge === null || poseAge === undefined ? "" : ` / ${currentFresh ? "实时" : "过期"} ${fmtAge(poseAge)}前`;
        addRow("地图位姿", `${fmtPose2d(payload.pose, 180 / Math.PI)}${ageText}`, poseCls);
      } else {
        addRow("地图位姿", poseUnavailableText(payload), poseCls);
      }
      if (navPose && Number.isFinite(Number(navPose.x)) && Number.isFinite(Number(navPose.y))) {
        const age = waypoint && Number.isFinite(Number(waypoint.nav_feedback_age_s)) ? ` / ${fmtAge(Number(waypoint.nav_feedback_age_s))}前` : "";
        addRow("Nav2反馈", `${fmtPose2d(navPose, 180 / Math.PI)}${age}`, "ok");
      } else {
        addRow("Nav2反馈", activeTask || waypoint ? "等待当前目标反馈" : "无活动任务", activeTask || waypoint ? "warn" : "");
      }
      if (goalPose && Number.isFinite(Number(goalPose.x)) && Number.isFinite(Number(goalPose.y))) {
        const goalName = wp && (wp.label || wp.id) ? ` / ${wp.label || wp.id}` : "";
        addRow("当前目标", `${fmtPose2d(goalPose)}${goalName}`, "ok");
      } else {
        addRow("当前目标", activeTask || waypoint ? "等待目标下发" : "无活动任务", activeTask || waypoint ? "warn" : "");
      }
      const robotGoalError = planarError(robotPose, goalPose);
      const navGoalError = planarError(navPose, goalPose);
      const robotNavError = planarError(robotPose, navPose);
      const metricParts = [];
      if (Number.isFinite(robotGoalError)) metricParts.push(`狗-目标 ${fmtNumber(robotGoalError, 2)}m`);
      if (Number.isFinite(navGoalError)) metricParts.push(`Nav2-目标 ${fmtNumber(navGoalError, 2)}m`);
      if (Number.isFinite(robotNavError)) metricParts.push(`狗-Nav2 ${fmtNumber(robotNavError, 2)}m`);
      if (waypoint && Number.isFinite(Number(waypoint.path_goal_error_m))) metricParts.push(`路径-目标 ${fmtNumber(Number(waypoint.path_goal_error_m), 2)}m`);
      if (waypoint && waypoint.plan_goal_verified === true) metricParts.push("路径已校验");
      addRow("误差", metricParts.length ? metricParts.join(" / ") : "等待任务目标和反馈", metricParts.length ? "ok" : "");
      const phaseParts = [];
      if (activeTask && activeTask.task_name) phaseParts.push(activeTask.task_name);
      if (waypoint && waypoint.phase) phaseParts.push(waypoint.phase === "dwelling" ? "停留中" : "导航中");
      if (waypoint && waypoint.nav_goal_status) phaseParts.push(`Nav2 ${waypoint.nav_goal_status}`);
      if (waypoint && Number.isFinite(Number(waypoint.distance_m))) phaseParts.push(`距离 ${fmtNumber(Number(waypoint.distance_m), 2)}m`);
      const runtimeGuard = (waypoint && waypoint.runtime_guard) || (activeTask && activeTask.runtime_guard) || null;
      if (runtimeGuard && runtimeGuard.code) phaseParts.push(`链路 ${runtimeGuard.code}`);
      addRow("任务阶段", phaseParts.length ? phaseParts.join(" / ") : "无活动任务", activeTask || waypoint ? "ok" : "");
      box.innerHTML = rows.join("");
    }
    async function loadPreflight() {
      try {
        const payload = await fetchJson("/api/preflight");
        const result = payload.preflight || null;
        renderPreflight(result);
        return result;
      } catch (err) {
        renderPreflight(null);
        return null;
      }
    }
    async function pollPreflightUntilDone(maxMs = 90000) {
      const deadline = Date.now() + maxMs;
      let result = null;
      while (Date.now() < deadline) {
        await sleepMs(1500);
        result = await loadPreflight();
        if (result && !result.running) return result;
      }
      throw {ok: false, message: "后台自检仍在执行，请刷新自检结果或查看 m20pro-real.service 日志"};
    }
    async function runPreflight() {
      const buttons = [$("runPreflightBtn"), $("taskRunPreflightBtn")].filter(Boolean);
      for (const btn of buttons) btn.disabled = true;
      if ($("preflightSummary")) $("preflightSummary").textContent = "基础自检中（工位/未重定位时只确认基础链路）...";
      if ($("taskPreflightSummary")) $("taskPreflightSummary").textContent = "基础自检中（工位/未重定位时只确认基础链路）...";
      try {
        const payload = await apiWithTimeout("POST", "/api/preflight/run", {mode: "move", site: "auto", wait: false}, 10000);
        const result = payload.preflight || payload;
        renderPreflight(result);
        if (payload.running || (result && result.running)) await pollPreflightUntilDone();
        await loadTasks();
      } catch (err) {
        renderPreflight({
          ok: false,
          navigation_ready: false,
          summary: err.message || "自检请求失败",
          age_sec: 0,
          items: [{
            key: "preflight_request",
            label: "自检请求",
            status: "fail",
            message: err.message || JSON.stringify(err)
          }]
        });
        setLog("preflightRaw", err);
      } finally {
        for (const btn of buttons) btn.disabled = false;
      }
    }
    function currentAnnotationMapId() {
      return state.selectedMapId || "live_map";
    }
    function asNumber(id, fallback) {
      const value = Number($(id).value);
      return Number.isFinite(value) ? value : fallback;
    }
    function asInteger(id, fallback) {
      const value = Number.parseInt($(id).value, 10);
      return Number.isFinite(value) ? value : fallback;
    }
    function syncManualDefaults(force) {
      const manualType = $("manualPointType").value;
      const defaults = defaultByManualType[manualType] || defaultByManualType.task;
      if (force || !$("markDwell").value.trim()) $("markDwell").value = String(defaults.dwell);
      if (force || !$("markGait").value.trim()) $("markGait").value = String(defaults.gait);
      if (force || !$("markSpeed").value.trim()) $("markSpeed").value = String(defaults.speed);
      if (force || !$("markManner").value.trim()) $("markManner").value = String(defaults.manner);
      if (force || !$("markObsMode").value.trim()) $("markObsMode").value = String(defaults.obsMode);
      if (force || !$("markNavMode").value.trim()) $("markNavMode").value = String(defaults.navMode);
    }
    async function fetchJson(url) {
      const res = await fetch(url, { cache: "no-store" });
      const payload = await res.json();
      if (!res.ok || payload.ok === false) throw payload;
      return payload;
    }
    async function api(method, url, body) {
      const res = await fetch(url, {
        method,
        headers: {"Content-Type": "application/json"},
        body: body === undefined ? undefined : JSON.stringify(body)
      });
      const payload = await res.json();
      if (!res.ok || payload.ok === false) throw payload;
      return payload;
    }
    async function apiWithTimeout(method, url, body, timeoutMs) {
      const controller = new AbortController();
      const timer = setTimeout(() => controller.abort(), timeoutMs);
      try {
        const res = await fetch(url, {
          method,
          headers: {"Content-Type": "application/json"},
          body: body === undefined ? undefined : JSON.stringify(body),
          signal: controller.signal
        });
        const payload = await res.json();
        if (!res.ok || payload.ok === false) throw payload;
        return payload;
      } catch (err) {
        if (err && err.name === "AbortError") {
          throw {ok: false, message: `请求超时：${Math.round(timeoutMs / 1000)} 秒内未收到网页返回；请刷新自检结果或检查 m20pro-real.service`};
        }
        throw err;
      } finally {
        clearTimeout(timer);
      }
    }
    function mapPreferredByFloor(floor) {
      if (!state.maps.length) return "";
      const normalized = String(floor || "").trim();
      if (normalized) {
        const byId = state.maps.find(item => item.id === `builtin_${normalized}`);
        if (byId) return byId.id;
        const byFloor = state.maps.find(item => item.floor === normalized);
        if (byFloor) return byFloor.id;
      }
      const f20 = state.maps.find(item => item.id === "builtin_F20") || state.maps.find(item => item.floor === "F20");
      if (f20) return f20.id;
      const builtin = state.maps.find(item => item.source === "project_builtin");
      return builtin ? builtin.id : state.maps[0].id;
    }
    function resizeCanvas() {
      const before = getView();
      const rect = canvas.parentElement.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.floor(rect.width * dpr));
      canvas.height = Math.max(1, Math.floor(rect.height * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      if (state.map && before && before.rect) {
        state.view.panX += (before.rect.width - rect.width) * 0.5;
        state.view.panY += (before.rect.height - rect.height) * 0.5;
        clampView();
      } else if (state.map) {
        state.view.panX = 0;
        state.view.panY = 0;
      }
      updateZoomReadout();
      draw();
    }
    function buildMapImage(map) {
      const image = document.createElement("canvas");
      image.width = map.width;
      image.height = map.height;
      const ictx = image.getContext("2d");
      const imageData = ictx.createImageData(map.width, map.height);
      for (let y = 0; y < map.height; y += 1) {
        for (let x = 0; x < map.width; x += 1) {
          const srcIdx = y * map.width + x;
          const flippedY = map.height - 1 - y;
          const dstIdx = (flippedY * map.width + x) * 4;
          const occ = map.data[srcIdx];
          let c = 205;
          if (occ >= 65) c = 0;
          else if (occ >= 0 && occ <= 25) c = 255;
          else if (occ >= 0) c = 150;
          imageData.data[dstIdx] = c;
          imageData.data[dstIdx + 1] = c;
          imageData.data[dstIdx + 2] = c;
          imageData.data[dstIdx + 3] = 255;
        }
      }
      ictx.putImageData(imageData, 0, 0);
      return image;
    }
    function getBaseView(rect = canvas.getBoundingClientRect()) {
      const map = state.map;
      if (!map) return { scale: 1, ox: 0, oy: 0, rect };
      const scale = Math.min(rect.width / map.width, rect.height / map.height);
      const drawW = map.width * scale;
      const drawH = map.height * scale;
      return { scale, ox: (rect.width - drawW) / 2, oy: (rect.height - drawH) / 2, rect };
    }
    function getView() {
      const base = getBaseView();
      const zoom = clampZoom(state.view.zoom);
      const scale = base.scale * zoom;
      const map = state.map;
      if (!map) return {...base, zoom: 1, baseScale: base.scale};
      const drawW = map.width * scale;
      const drawH = map.height * scale;
      return {
        scale,
        baseScale: base.scale,
        zoom,
        ox: (base.rect.width - drawW) / 2 + state.view.panX,
        oy: (base.rect.height - drawH) / 2 + state.view.panY,
        rect: base.rect
      };
    }
    function clampZoom(value) {
      const zoom = Number(value);
      if (!Number.isFinite(zoom)) return 1;
      return Math.max(0.25, Math.min(12, zoom));
    }
    function updateZoomReadout(view = getView()) {
      if (!$("zoomReadout")) return;
      $("zoomReadout").textContent = `${Math.round((view.zoom || state.view.zoom || 1) * 100)}%`;
    }
    function clampView() {
      if (!state.map) return;
      state.view.zoom = clampZoom(state.view.zoom);
      const view = getView();
      const drawW = state.map.width * view.scale;
      const drawH = state.map.height * view.scale;
      const margin = 80;
      if (drawW <= view.rect.width) {
        state.view.panX = 0;
      } else {
        const limitX = (drawW - view.rect.width) * 0.5 + margin;
        state.view.panX = Math.max(-limitX, Math.min(limitX, state.view.panX));
      }
      if (drawH <= view.rect.height) {
        state.view.panY = 0;
      } else {
        const limitY = (drawH - view.rect.height) * 0.5 + margin;
        state.view.panY = Math.max(-limitY, Math.min(limitY, state.view.panY));
      }
      updateZoomReadout();
    }
    function resetMapView(redraw = true) {
      state.view.zoom = 1;
      state.view.panX = 0;
      state.view.panY = 0;
      updateZoomReadout();
      if (redraw) draw();
    }
    function setZoomAt(clientX, clientY, nextZoom) {
      if (!state.map) return;
      const oldView = getView();
      const rect = oldView.rect;
      const cx = clientX - rect.left;
      const cy = clientY - rect.top;
      const mx = (cx - oldView.ox) / oldView.scale;
      const my = (cy - oldView.oy) / oldView.scale;
      state.view.zoom = clampZoom(nextZoom);
      const newView = getView();
      state.view.panX += cx - (newView.ox + mx * newView.scale);
      state.view.panY += cy - (newView.oy + my * newView.scale);
      clampView();
      draw();
    }
    function zoomBy(factor) {
      const rect = canvas.getBoundingClientRect();
      setZoomAt(rect.left + rect.width * 0.5, rect.top + rect.height * 0.5, state.view.zoom * factor);
    }
    function updateMapModeUi() {
      $("mapMode").textContent = state.mapModeLabel || "实时 /map";
      $("cursor").textContent = state.view.panMode ? "平移模式" : "拖拽地图取点和朝向";
      updateFloorDisplay();
      updateZoomReadout();
    }
    function centerMapOnWorld(x, y) {
      if (!state.map || !Number.isFinite(Number(x)) || !Number.isFinite(Number(y))) return;
      const view = getView();
      const target = worldToCanvasWithView(Number(x), Number(y), view);
      if (!target) return;
      state.view.panX += view.rect.width * 0.5 - target.x;
      state.view.panY += view.rect.height * 0.5 - target.y;
      clampView();
      draw();
    }
    function followRobotIfNeeded(activeTask) {
      const pose = freshPose();
      if (!state.followRobot || !activeTask || !pose || state.view.panMode || !isViewingRobotFloor(state.latest)) return;
      centerMapOnWorld(pose.x, pose.y);
    }
    function worldToCanvasWithView(x, y, view) {
      const map = state.map;
      if (!map) return null;
      const mx = (x - map.origin.x) / map.resolution;
      const my = map.height - (y - map.origin.y) / map.resolution;
      return { x: view.ox + mx * view.scale, y: view.oy + my * view.scale };
    }
    function worldToCanvas(x, y) {
      return worldToCanvasWithView(x, y, getView());
    }
    function canvasToWorld(clientX, clientY) {
      const map = state.map;
      if (!map) return null;
      const rect = canvas.getBoundingClientRect();
      const view = getView();
      const cx = clientX - rect.left;
      const cy = clientY - rect.top;
      const mx = (cx - view.ox) / view.scale;
      const my = (cy - view.oy) / view.scale;
      if (mx < 0 || my < 0 || mx > map.width || my > map.height) return null;
      return {
        x: map.origin.x + mx * map.resolution,
        y: map.origin.y + (map.height - my) * map.resolution
      };
    }
    function normalizeYaw(yaw) {
      let value = Number(yaw);
      if (!Number.isFinite(value)) return 0;
      while (value > Math.PI) value -= Math.PI * 2;
      while (value <= -Math.PI) value += Math.PI * 2;
      return value;
    }
    function currentMarkYaw() {
      return normalizeYaw($("markYaw").value);
    }
    function currentLocalizeYaw() {
      return normalizeYaw($("locYaw").value);
    }
    function setMarkDraft(pose, message, source = "map_click") {
      state.markDraft = {
        x: Number(pose.x),
        y: Number(pose.y),
        yaw: normalizeYaw(pose.yaw)
      };
      state.markDraftSource = source;
      const shownFloor = displayedMapFloor();
      if (source === "map_click" && shownFloor && $("markFloor")) $("markFloor").value = shownFloor;
      $("markXY").value = `${state.markDraft.x.toFixed(3)}, ${state.markDraft.y.toFixed(3)}`;
      $("markYaw").value = state.markDraft.yaw.toFixed(4);
      $("cursor").textContent = message || `x ${state.markDraft.x.toFixed(3)} / y ${state.markDraft.y.toFixed(3)} / 朝向 ${state.markDraft.yaw.toFixed(3)} rad`;
      updateMarkControls();
      draw();
    }
    function setLocalizeDraft(pose, message) {
      state.localizeDraft = {
        x: Number(pose.x),
        y: Number(pose.y),
        yaw: normalizeYaw(pose.yaw)
      };
      $("locXY").value = `${state.localizeDraft.x.toFixed(3)}, ${state.localizeDraft.y.toFixed(3)}`;
      $("locYaw").value = state.localizeDraft.yaw.toFixed(4);
      $("cursor").textContent = message || `定位 x ${state.localizeDraft.x.toFixed(3)} / y ${state.localizeDraft.y.toFixed(3)} / 朝向 ${state.localizeDraft.yaw.toFixed(3)} rad`;
      draw();
    }
    function activeTabName() {
      const active = document.querySelector("button.tab.active");
      return active ? active.dataset.tab : "";
    }
    function drawArrow(pose, options = {}) {
      if (!Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) return;
      const p = worldToCanvas(pose.x, pose.y);
      if (!p) return;
      const color = options.color || "#0f6bff";
      const size = options.size || 1.0;
      const label = options.label || "";
      ctx.save();
      ctx.translate(p.x, p.y);
      ctx.rotate(-(Number(pose.yaw) || 0));
      ctx.fillStyle = color;
      ctx.strokeStyle = options.stroke || "#ffffff";
      ctx.lineWidth = options.lineWidth || 2;
      ctx.beginPath();
      ctx.moveTo(15 * size, 0);
      ctx.lineTo(-10 * size, -8 * size);
      ctx.lineTo(-6 * size, 0);
      ctx.lineTo(-10 * size, 8 * size);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      if (label) {
        ctx.save();
        ctx.font = "12px system-ui, sans-serif";
        ctx.fillStyle = "#17212b";
        ctx.fillText(label, p.x + 11, p.y - 9);
        ctx.restore();
      }
    }
    function drawPath(path) {
      if (!path || !path.points || path.points.length < 2) return;
      ctx.save();
      ctx.strokeStyle = "#f97316";
      ctx.lineWidth = 3;
      ctx.beginPath();
      let started = false;
      for (const point of path.points) {
        const p = worldToCanvas(point.x, point.y);
        if (!p) continue;
        if (!started) { ctx.moveTo(p.x, p.y); started = true; }
        else ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
      ctx.restore();
    }
    function drawPoseHistory(points) {
      if (!Array.isArray(points) || points.length < 2 || !state.map) return;
      const view = getView();
      ctx.save();
      ctx.strokeStyle = "#0f766e";
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 4]);
      ctx.beginPath();
      let started = false;
      for (const point of points) {
        if (!Number.isFinite(Number(point.x)) || !Number.isFinite(Number(point.y))) continue;
        const p = worldToCanvasWithView(point.x, point.y, view);
        if (!p) continue;
        if (!started) { ctx.moveTo(p.x, p.y); started = true; }
        else ctx.lineTo(p.x, p.y);
      }
      if (started) ctx.stroke();
      ctx.restore();
    }
    function drawObstacles(items) {
      if (!items || items.length === 0 || !state.map) return;
      const view = getView();
      ctx.save();
      for (const item of items) {
        const p = worldToCanvasWithView(item.x, item.y, view);
        if (!p) continue;
        const radius = Math.max(5, Math.min(22, (item.scale_x || 0.4) / state.map.resolution * view.scale * 0.5));
        ctx.fillStyle = "rgba(185, 28, 28, 0.82)";
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }
      ctx.restore();
    }
    function drawAnnotations() {
      if (!state.annotations || !state.map) return;
      for (const item of state.annotations) {
        const pose = item.pose || {};
        const color = typeColors[item.type] || "#0f6bff";
        const label = item.label || typeNames[item.type] || "point";
        drawArrow(
          {x: Number(pose.x), y: Number(pose.y), yaw: Number(pose.yaw) || 0},
          {color, size: 0.72, label}
        );
      }
    }
    function drawMarkDraft() {
      if (!state.markDraft) return;
      drawArrow(state.markDraft, {
        color: "#16a34a",
        stroke: "#f8fafc",
        lineWidth: 2.5,
        size: 0.92,
        label: "待保存"
      });
    }
    function drawLocalizeDraft() {
      if (!state.localizeDraft) return;
      if (localizationConfirmedForDisplay()) return;
      drawArrow(state.localizeDraft, {
        color: "#dc2626",
        stroke: "#fef2f2",
        lineWidth: 2.5,
        size: 1.0,
        label: "定位"
      });
    }
    function drawActiveWaypointTarget() {
      const waypoint = state.latest
        && state.latest.active_waypoint
        && state.latest.active_waypoint.parsed
        && state.latest.active_waypoint.parsed.waypoint;
      const pose = waypoint && waypoint.pose;
      if (!pose) return;
      const waypointFloor = waypoint.floor ? String(waypoint.floor).trim() : "";
      const shownFloor = displayedMapFloor();
      if (shownFloor && waypointFloor && shownFloor !== waypointFloor) return;
      drawArrow(
        {x: Number(pose.x), y: Number(pose.y), yaw: Number(pose.yaw) || 0},
        {
          color: "#f97316",
          stroke: "#fff7ed",
          lineWidth: 3,
          size: 1.08,
          label: `目标 ${waypoint.label || ""}`.trim()
        }
      );
    }
    function drawNavFeedbackPose() {
      if (!isViewingRobotFloor()) return;
      const active = state.latest && state.latest.active_task;
      const waypoint = state.latest
        && state.latest.active_waypoint
        && state.latest.active_waypoint.parsed;
      const feedback = (waypoint && waypoint.nav_feedback) || (active && active.last_nav_feedback) || null;
      if (!feedback || !Number.isFinite(Number(feedback.pose_x)) || !Number.isFinite(Number(feedback.pose_y))) return;
      drawArrow(
        {
          x: Number(feedback.pose_x),
          y: Number(feedback.pose_y),
          yaw: Number.isFinite(Number(feedback.pose_yaw)) ? Number(feedback.pose_yaw) : 0
        },
        {
          color: "#7c3aed",
          stroke: "#f5f3ff",
          lineWidth: 2,
          size: 0.84,
          label: "Nav2反馈"
        }
      );
    }
    function drawScanOverlay() {
      if (!state.scanOverlay || !state.map || !state.latest || !state.latest.scan) return;
      const points = state.latest.scan.points || [];
      if (!points.length) return;
      const usingDraft = activeTabName() === "localize" && state.localizeDraft && !localizationConfirmedForDisplay();
      if (!usingDraft && !isViewingRobotFloor()) return;
      if (usingDraft && !isViewingRobotFloor()) return;
      const pose = usingDraft ? state.localizeDraft : freshPose();
      if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) return;
      const yaw = normalizeYaw(pose.yaw || 0);
      const cosYaw = Math.cos(yaw);
      const sinYaw = Math.sin(yaw);
      const offset = state.latest.scan_overlay_offset || {};
      const offX = Number(offset.x || 0);
      const offY = Number(offset.y || 0);
      const offYaw = normalizeYaw(offset.yaw || 0);
      const cosOff = Math.cos(offYaw);
      const sinOff = Math.sin(offYaw);
      const view = getView();
      ctx.save();
      ctx.fillStyle = usingDraft ? "rgba(220, 38, 38, 0.72)" : "rgba(14, 165, 233, 0.78)";
      const radius = Math.max(1.4, Math.min(3.2, view.scale * 1.6));
      for (const point of points) {
        const px = Number(point.x);
        const py = Number(point.y);
        if (!Number.isFinite(px) || !Number.isFinite(py)) continue;
        const bx = offX + cosOff * px - sinOff * py;
        const by = offY + sinOff * px + cosOff * py;
        const wx = Number(pose.x) + cosYaw * bx - sinYaw * by;
        const wy = Number(pose.y) + sinYaw * bx + cosYaw * by;
        const p = worldToCanvasWithView(wx, wy, view);
        if (!p) continue;
        ctx.fillRect(p.x - radius * 0.5, p.y - radius * 0.5, radius, radius);
      }
      ctx.restore();
    }
    function draw() {
      const rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      ctx.fillStyle = "#cfd5dc";
      ctx.fillRect(0, 0, rect.width, rect.height);
      if (!state.map || !state.mapImage) {
        ctx.fillStyle = "#667483";
        ctx.font = "15px system-ui, sans-serif";
        ctx.fillText("等待地图数据", 20, 30);
        return;
      }
      const view = getView();
      updateZoomReadout(view);
      ctx.drawImage(state.mapImage, view.ox, view.oy, state.map.width * view.scale, state.map.height * view.scale);
      ctx.strokeStyle = "#4b5563";
      ctx.lineWidth = 1;
      ctx.strokeRect(view.ox, view.oy, state.map.width * view.scale, state.map.height * view.scale);
      const latest = state.latest;
      const canDrawLiveRobotLayer = latest && isViewingRobotFloor(latest);
      if (!canDrawLiveRobotLayer || !hasFreshPose(latest)) state.robotDisplayPose = null;
      if (canDrawLiveRobotLayer) {
        drawPath(latest.path);
        drawPoseHistory(latest.pose_history || []);
        drawObstacles(latest.dynamic_obstacles);
      }
      drawScanOverlay();
      drawAnnotations();
      drawActiveWaypointTarget();
      drawNavFeedbackPose();
      drawMarkDraft();
      drawLocalizeDraft();
      if (canDrawLiveRobotLayer && hasFreshPose(latest)) {
        const robotPose = stableRobotDisplayPose(latest.pose, latest.active_task || null);
        if (robotPose) drawArrow(robotPose);
      }
    }
    async function refreshLiveMap(version) {
      if (state.selectedMapId || version === state.liveMapVersion) return;
      const map = await fetchJson("/api/map");
      if (!map.available) return;
      const resetView = !state.map || state.map.width !== map.width || state.map.height !== map.height;
      state.map = map;
      state.mapImage = buildMapImage(map);
      state.selectedMapId = null;
      state.liveMapVersion = map.version;
      $("mapTitle").textContent = `实时地图版本 ${map.version}`;
      $("mapMeta").textContent = `${map.width} x ${map.height}, ${map.resolution.toFixed(3)} m/格`;
      state.mapModeLabel = "实时 /map";
      updateMapModeUi();
      updateMarkControls();
      await loadAnnotations();
      if (resetView) resetMapView(false);
      resizeCanvas();
    }
    async function loadFileMap(mapId) {
      if (!mapId) {
        state.selectedMapId = null;
        state.fileMapVersion = -1;
        state.map = null;
        state.mapImage = null;
        $("mapTitle").textContent = "实时地图";
        $("mapMeta").textContent = "等待 /map 数据";
        state.mapModeLabel = "实时 /map";
        updateMapModeUi();
        updateMarkControls();
        return;
      }
      const map = await fetchJson(`/api/map_file?map_id=${encodeURIComponent(mapId)}`);
      if (!map.available) {
        const message = map.message || `地图 ${mapId} 不可用`;
        $("mapTitle").textContent = "固定地图加载失败";
        $("mapMeta").textContent = message;
        $("cursor").textContent = message;
        throw {ok: false, message};
      }
      state.map = map;
      state.mapImage = buildMapImage(map);
      state.selectedMapId = mapId;
      state.fileMapVersion = map.version;
      state.markDraft = null;
      state.markDraftSource = "map_click";
      const select = $("mapSelect");
      if (select && select.value !== mapId) select.value = mapId;
      $("mapTitle").textContent = map.name || `固定地图 ${mapId}`;
      $("mapMeta").textContent = `${map.floor || "-"} / ${map.width} x ${map.height}, ${map.resolution.toFixed(3)} m/格`;
      if ($("markFloor") && map.floor) $("markFloor").value = map.floor;
      state.mapModeLabel = map.source === "project_builtin" ? "项目内置地图" : "固定地图";
      updateMapModeUi();
      await loadAnnotations();
      updateMarkControls();
      resetMapView(false);
      resizeCanvas();
    }
    function updateState(s) {
      state.latest = s;
      state.selectedMapStatus = s.selected_map_status || null;
      if (state.localizeDraft && localizationConfirmedForDisplay(s)) {
        state.localizeDraft = null;
      }
      renderTaskReadiness(s.task_readiness || null);
      renderLocalizationStatus(s);
      updateFloorDisplay(s);
      updateMarkControls(s);
      $("floor").textContent = text(s.floor);
      $("stair").textContent = text(s.stair_status);
      const gaitParts = [];
      if (s.usage_mode_result) gaitParts.push(text(s.usage_mode_result));
      if (s.gait_result) gaitParts.push(text(s.gait_result));
      else if (s.gait_command) gaitParts.push(text(s.gait_command));
      const usageMode = s.navigation_status_parsed ? s.navigation_status_parsed.usage_mode : null;
      const ooa = s.navigation_status_parsed ? s.navigation_status_parsed.ooa : null;
      const usageModeText = formatUsageMode(usageMode);
      const ooaText = formatOoa(ooa);
      if (usageModeText) gaitParts.push(usageModeText);
      if (ooaText) gaitParts.push(ooaText);
      $("gait").textContent = gaitParts.length ? gaitParts.join(" / ") : "-";
      const currentPoseFresh = hasFreshPose(s);
      const currentPoseAge = poseAgeSec(s);
      if (s.pose) {
        const yawDeg = Number.isFinite(Number(s.pose.display_yaw_deg)) ? s.pose.display_yaw_deg : s.pose.yaw_deg;
        const rawYaw = fmtNumber(s.pose.yaw_deg, 0);
        const shownYaw = fmtNumber(yawDeg, 0);
        const offsetDeg = Number(s.pose.display_yaw_offset_deg || 0);
        const offsetText = Math.abs(offsetDeg) > 0.01 ? ` / 显示偏置 ${fmtNumber(offsetDeg, 0)}°` : "";
        const ageText = currentPoseAge === null || currentPoseAge === undefined ? "" : ` / ${currentPoseFresh ? "实时" : "最后"} ${fmtAge(currentPoseAge)}前`;
        $("pose").textContent = `x ${fmtNumber(s.pose.x)} / y ${fmtNumber(s.pose.y)} / 朝向 ${shownYaw}° / 原始 ${rawYaw}°${offsetText}${ageText}`;
      }
      else {
        const navPose = latestNavFeedbackPose(s);
        if (navPose) {
          const ageText = navPose.age_s === null || navPose.age_s === undefined ? "" : ` / ${fmtAge(navPose.age_s)}前`;
          $("pose").textContent = `Nav2反馈 x ${fmtNumber(navPose.x)} / y ${fmtNumber(navPose.y)} / 朝向 ${fmtNumber(navPose.yaw * 180 / Math.PI, 0)}°${ageText}`;
        } else {
          $("pose").textContent = poseUnavailableText(s);
        }
      }
      if (s.localization_ok === true && currentPoseFresh) $("localization").textContent = "正常";
      else if (s.localization_ok === true) $("localization").textContent = "位姿过期";
      else if (s.localization_ok === false) $("localization").textContent = "异常/未定位";
      else $("localization").textContent = "-";
      $("factoryNav").textContent = text(s.navigation_status);
      renderPoseTracker("livePoseTracker", s);
      renderPoseTracker("taskPoseTracker", s);
      if ($("scanOverlayStatus")) {
        const scan = s.scan || {};
        const points = scan.points || [];
        if (points.length) {
          const age = scan.last_update ? Math.max(0, s.node_time - scan.last_update) : null;
          const floorMismatch = selectedMapFloorMismatchText(s);
          const mode = floorMismatch
            ? `${floorMismatch}，暂停叠加`
            : activeTabName() === "localize" && state.localizeDraft && !localizationConfirmedForDisplay(s)
            ? "红色=待重定位预览"
            : (currentPoseFresh ? "蓝色=当前位姿" : "当前位姿未确认，暂停叠加");
          $("scanOverlayStatus").textContent = `激光轮廓 ${points.length} 点 / ${mode} / ${fmtAge(age)}前`;
        } else if (scan.finite_ranges) {
          $("scanOverlayStatus").textContent = `收到 /scan，但无可绘制轮廓点`;
        } else {
          $("scanOverlayStatus").textContent = "等待 /scan 数据";
        }
      }
      if (s.battery && s.battery.primary) {
        const pack = s.battery.primary;
        const tempText = Number.isFinite(Number(pack.temperature_c)) ? ` / ${fmtNumber(Number(pack.temperature_c), 1)}℃` : "";
        $("battery").textContent = `${text(pack.level)}% / ${fmtNumber(Number(pack.voltage_v), 1)}V / ${fmtNumber(Number(pack.current_a), 1)}A${tempText}`;
      } else {
        $("battery").textContent = "-";
      }
      $("nav").textContent = JSON.stringify({
        路径点数: s.path ? s.path.points.length : 0,
        动态障碍物: s.dynamic_obstacles ? s.dynamic_obstacles.length : 0,
        感知链路: s.perception_status || null,
        当前任务: s.active_task || null,
        当前点位: s.active_waypoint || null,
        电量: s.battery && s.battery.primary ? s.battery.primary : null,
        定位状态: s.localization_ok,
        原厂导航: s.navigation_status || null,
        更新时间: s.node_time_text
      }, null, 2);
      const det = s.detections && (s.detections.parsed || s.detections.raw);
      $("detections").textContent = det ? JSON.stringify(det, null, 2) : "等待数据";
      $("events").textContent = s.events && s.events.length ? JSON.stringify(s.events.slice(-5), null, 2) : "等待数据";
      if (
        s.relocalization_result
        && s.relocalization_result.last_update !== state.lastRelocalizationStamp
        && Date.now() > state.relocalizationApiLogUntil
      ) {
        state.lastRelocalizationStamp = s.relocalization_result.last_update;
        setLog("localizeLog", {
          "2101原始回执": s.relocalization_result.raw,
          定位结论: s.localization_status ? s.localization_status.message : null,
          任务页: s.localization_status && s.localization_status.task_ready ? "可启动" : "不可启动",
          更新时间: s.node_time_text
        });
      }
      if (s.active_task || s.active_waypoint) {
        state.activeTaskLogUntil = 0;
        const waypoint = s.active_waypoint && s.active_waypoint.parsed ? s.active_waypoint.parsed : null;
        followRobotIfNeeded(s.active_task || null);
        renderActiveTaskSummary(s.active_task || null, waypoint || null);
        $("activeTask").textContent = JSON.stringify({
          task: s.active_task || null,
          waypoint,
          stair_status: s.stair_status || null,
          navigation_status: s.navigation_status || null
        }, null, 2);
      } else if (Date.now() > state.activeTaskLogUntil) {
        renderActiveTaskSummary(null, null);
        $("activeTask").textContent = "无任务";
      }
      updateTaskControlButtons(s);
      const table = $("topics");
      table.innerHTML = "";
      for (const [name, info] of Object.entries(s.topics || {})) {
        const tr = document.createElement("tr");
        const left = document.createElement("td");
        const right = document.createElement("td");
        left.textContent = name;
        right.textContent = info.available ? fmtAge(info.age_sec) : "无数据";
        tr.appendChild(left);
        tr.appendChild(right);
        table.appendChild(tr);
      }
    }
    async function loadMaps() {
      const payload = await fetchJson("/api/maps");
      state.maps = payload.maps || [];
      const selected = payload.selected_map_id || mapPreferredByFloor(
        (state.latest && state.latest.floor) || $("locFloor").value || "F20"
      );
      const select = $("mapSelect");
      select.innerHTML = "";
      const live = document.createElement("option");
      live.value = "";
      live.textContent = "实时 /map";
      select.appendChild(live);
      for (const map of state.maps) {
        const opt = document.createElement("option");
        opt.value = map.id;
        const sourceText = map.source === "project_builtin" ? "项目内置" : "106归档";
        opt.textContent = `${map.name || map.id} (${map.floor || "-"} / ${sourceText})`;
        select.appendChild(opt);
      }
      select.value = selected;
      if (selected && selected !== state.selectedMapId) {
        try {
          const result = await api("POST", "/api/maps/select", {map_id: selected});
          setLog("mapsLog", result);
        } catch (err) {
          console.warn(err);
        }
        await loadFileMap(selected);
      } else if (!selected && state.selectedMapId) {
        await loadFileMap("");
      }
      updateMarkControls();
      renderMapList();
    }
    function renderMapList() {
      const box = $("mapList");
      box.innerHTML = "";
      if (!state.maps.length) {
        box.innerHTML = `<div class="small">当前没有可选固定地图，可先使用实时 /map 或从 106 拉取地图。</div>`;
        return;
      }
      for (const map of state.maps) {
        const el = document.createElement("div");
        el.className = "item";
        const sourceText = map.source === "project_builtin" ? "项目内置" : "106 归档";
        const note = map.source_note ? `<br>${map.source_note}` : "";
        el.innerHTML = `
          <div class="item-head"><span>${map.name || map.id}</span><span class="tag">${map.floor || "-"}</span></div>
          <div class="item-meta">${sourceText} / ${map.yaml_path || ""}<br>${map.created_at || ""}${note}</div>
        `;
        box.appendChild(el);
      }
    }
    async function loadAnnotations() {
      const mapId = currentAnnotationMapId();
      const payload = await fetchJson(`/api/annotations${mapId ? `?map_id=${encodeURIComponent(mapId)}` : ""}`);
      state.annotations = payload.annotations || [];
      renderAnnotations();
      renderTaskPoints();
    }
    function renderAnnotations() {
      const box = $("annotationList");
      box.innerHTML = "";
      if (!state.annotations.length) {
        box.innerHTML = `<div class="small">当前地图还没有点位。</div>`;
        return;
      }
      for (const item of state.annotations) {
        const pose = item.pose || {};
        const place = [item.area, item.room, item.result_file_prefix ? `结果:${item.result_file_prefix}` : ""].filter(Boolean).join(" / ");
        const el = document.createElement("div");
        el.className = "item";
        el.innerHTML = `
          <div class="item-head">
            <span>${item.label || typeNames[item.type] || item.id}</span>
            <span class="tag">${typeNames[item.type] || item.type}</span>
          </div>
          <div class="item-meta">${item.floor || "-"} / ${manualPointTypeNames[item.manual_point_type] || item.manual_point_type || "-"} / x ${fmtNumber(Number(pose.x))}, y ${fmtNumber(Number(pose.y))}, 朝向 ${fmtNumber(Number(pose.yaw), 2)} / 停留 ${fmtNumber(Number(item.dwell_s || 0), 1)}s</div>
          ${place ? `<div class="item-meta">${place}</div>` : ""}
          <div class="actions"><button class="danger" data-delete-mark="${item.id}">删除</button></div>
        `;
        box.appendChild(el);
      }
      for (const btn of box.querySelectorAll("[data-delete-mark]")) {
        btn.addEventListener("click", async () => {
          await api("DELETE", `/api/annotations?id=${encodeURIComponent(btn.dataset.deleteMark)}`);
          await loadAnnotations();
          draw();
        });
      }
    }
    function renderTaskPoints() {
      const box = $("taskPointList");
      if (!state.annotations.length) {
        box.textContent = "请先选择地图并标点";
        updateCreateTaskButton();
        renderTaskNextStep();
        return;
      }
      box.innerHTML = "";
      for (const item of state.annotations) {
        const line = document.createElement("label");
        line.className = "checkline";
        const place = [item.area, item.room].filter(Boolean).join(" / ");
        line.innerHTML = `<input type="checkbox" value="${item.id}"><span>${item.floor || "-"} / ${item.label || item.id}${place ? ` / ${place}` : ""} / ${manualPointTypeNames[item.manual_point_type] || item.manual_point_type || typeNames[item.type] || item.type} / 朝向 ${fmtNumber(Number((item.pose || {}).yaw), 2)} / 停留 ${fmtNumber(Number(item.dwell_s || 0), 1)}s</span>`;
        box.appendChild(line);
      }
      for (const input of box.querySelectorAll("input[type='checkbox']")) {
        input.addEventListener("change", updateCreateTaskButton);
      }
      updateCreateTaskButton();
      renderTaskNextStep();
    }
    function updateCreateTaskButton() {
      const btn = $("createTaskBtn");
      const box = $("taskPointList");
      if (!btn || !box) return;
      const selectedMapStatus = state.selectedMapStatus || {};
      if (selectedMapStatus.ready === false) {
        btn.disabled = true;
        btn.title = selectedMapStatus.message || "网页选择地图与 Nav2 当前加载地图不一致，请先切换到正确地图并重定位";
        return;
      }
      const total = box.querySelectorAll("input[type='checkbox']").length;
      const checked = box.querySelectorAll("input[type='checkbox']:checked").length;
      if (!total) {
        btn.disabled = true;
        btn.title = "当前地图还没有任务点；先在当前地图标点";
        return;
      }
      if (!checked) {
        btn.disabled = true;
        btn.title = "先勾选当前地图点位";
        return;
      }
      btn.disabled = false;
      btn.title = "用已勾选的当前地图点位生成任务";
    }
    async function loadTasks() {
      if (state.loadingTasks) return;
      state.loadingTasks = true;
      try {
        const payload = await fetchJson("/api/tasks");
        state.lastTasksRefreshAt = Date.now();
        state.lastTasksPayload = payload;
        if (payload.selected_map_status) state.selectedMapStatus = payload.selected_map_status;
        if (payload.task_readiness) renderTaskReadiness(payload.task_readiness);
        state.tasks = payload.tasks || [];
        renderTaskNextStep();
        const box = $("taskList");
        box.innerHTML = "";
        if (!state.tasks.length) {
          const hiddenOldTaskCount = Number(payload.hidden_task_count || 0);
          const hiddenText = hiddenOldTaskCount > 0 ? `旧地图任务已隐藏 ${hiddenOldTaskCount} 个，` : "";
          box.innerHTML = `<div class="preflight-summary warn">当前地图还没有任务；${hiddenText}只保留为历史审计，默认接口不会返回，不能用于本次现场执行。请先在当前地图标点并生成任务。</div>`;
          return;
        }
        const currentMapTasks = state.tasks.filter(task => taskBelongsToSelectedMap(task));
        if (!currentMapTasks.length) {
          const notice = document.createElement("div");
          notice.className = "preflight-summary warn";
          const hiddenOldTaskCount = Number(payload.hidden_task_count || (state.tasks.length - currentMapTasks.length));
          const hiddenText = hiddenOldTaskCount > 0 ? `旧地图任务已隐藏 ${hiddenOldTaskCount} 个，` : "";
          notice.textContent = `当前地图还没有任务；${hiddenText}只保留为历史审计，默认接口不会返回，不能用于本次现场执行。请先在当前地图标点并生成任务。`;
          box.appendChild(notice);
        }
        for (const task of currentMapTasks) {
          const activeTask = payload.active_task && payload.active_task.status === "running" ? payload.active_task : null;
          const active = !!activeTask;
          const isRunning = !!(activeTask && activeTask.task_id === task.id);
          const readiness = task.readiness || {};
          const taskStatus = normalizedTaskStatus(task.status);
          const statusAllowsStart = taskStatusAllowsStart(taskStatus);
          const mapMismatchText = taskMapMismatchText(task);
          const canStart = !active && !isRunning && statusAllowsStart && readiness.ready === true && !mapMismatchText;
          const canDelete = !isRunning && !(payload.active_task && payload.active_task.task_id === task.id);
          const canCopyEvidence = !mapMismatchText;
          const statusBlockText = taskStatusBlockText(taskStatus);
          const startLabel = isRunning ? "执行中" : (active ? "先停止当前任务" : (mapMismatchText ? "旧地图任务" : taskStartLabelForStatus(taskStatus, readiness)));
          const readinessText = mapMismatchText || statusBlockText || readiness.message || "等待任务条件检查";
          const displayStatus = taskDisplayStatus(taskStatus, readiness, mapMismatchText);
          const firstDistanceText = taskFirstDistanceText(readiness);
          const waypointDetails = Array.isArray(task.waypoints) ? task.waypoints : [];
          const waypointOrderText = waypointDetails.length
            ? waypointDetails.map((point, index) => taskWaypointText(point, index)).join(" → ")
            : "无点位";
          const firstWaypoint = waypointDetails.length ? waypointDetails[0] : null;
          const firstPose = firstWaypoint && firstWaypoint.pose ? firstWaypoint.pose : {};
          const firstTargetText = firstWaypoint
            ? `首点：${firstWaypoint.floor || "-"} / ${firstWaypoint.label || firstWaypoint.id || "-"} / x ${fmtNumber(Number(firstPose.x), 2)}, y ${fmtNumber(Number(firstPose.y), 2)}, 朝向 ${fmtNumber(Number(firstPose.yaw), 2)}`
            : "首点：-";
          const readyCheckCommand = taskReadyCheckCommand(task);
          const watcherCommand = taskWatcherCommand(task);
          const evidenceCommandHtml = canCopyEvidence ? `
            <div class="item-meta">开跑前验收：${readyCheckCommand}</div>
            <div class="item-meta">开跑前记录：${watcherCommand}</div>
          ` : "";
          const evidenceButtonHtml = canCopyEvidence ? `
              <button data-copy-command="${readyCheckCommand}" title="复制任务专属 ready-check 命令">复制验收</button>
              <button data-copy-command="${watcherCommand}" title="复制任务专属 watcher 命令">复制记录</button>
          ` : "";
          const lastResultText = taskLastResultText(task);
          const el = document.createElement("div");
          el.className = "item";
          el.innerHTML = `
            <div class="item-head"><span>${task.name || task.id}</span><span class="tag">${displayStatus}</span></div>
            <div class="item-meta">${(task.annotation_ids || []).length} 个点 / ${task.created_at || ""}${task.updated_at ? ` / 更新 ${task.updated_at}` : ""}</div>
            <div class="item-meta">${firstTargetText}</div>
            ${firstDistanceText ? `<div class="item-meta">${firstDistanceText}</div>` : ""}
            <div class="item-meta">顺序：${waypointOrderText}</div>
            <div class="item-meta">执行条件：${readinessText}</div>
            ${evidenceCommandHtml}
            ${lastResultText ? `<div class="item-meta">${lastResultText}</div>` : ""}
            <div class="actions">
              ${evidenceButtonHtml}
              <button class="primary" data-start-task="${task.id}" title="${readinessText}" ${canStart ? "" : "disabled"}>${startLabel}</button>
              <button data-rename-task="${task.id}">改名</button>
              <button class="danger" data-delete-task="${task.id}" ${canDelete ? "" : "disabled"}>删除</button>
            </div>
          `;
          box.appendChild(el);
        }
        for (const btn of box.querySelectorAll("[data-start-task]")) {
          btn.addEventListener("click", async () => {
            btn.disabled = true;
            btn.textContent = "确认中...";
            try {
              const task = state.tasks.find(item => item.id === btn.dataset.startTask);
              if (!task) throw {message: "任务列表已变化，请刷新后重试"};
              const readiness = task.readiness || {};
              const readinessText = taskStatusBlockText(normalizedTaskStatus(task.status)) || readiness.message || "等待任务条件检查";
              if (!window.confirm(taskStartConfirmText(task, readinessText))) return;
              btn.textContent = "启动中...";
              const payload = await api("POST", "/api/tasks/start", taskStartRequest(task));
              state.activeTaskLogUntil = Date.now() + 20000;
              setLog("activeTask", payload.active_task || payload);
              await loadTasks();
            } catch (err) {
              state.activeTaskLogUntil = Date.now() + 30000;
              setLog("activeTask", err);
            } finally {
              await loadTasks();
            }
          });
        }
        for (const btn of box.querySelectorAll("[data-copy-command]")) {
          btn.addEventListener("click", async () => {
            const original = btn.textContent;
            try {
              const ok = await copyTextToClipboard(btn.dataset.copyCommand || "");
              btn.textContent = ok ? "已复制" : "复制失败";
            } catch (err) {
              btn.textContent = "复制失败";
            }
            setTimeout(() => { btn.textContent = original; }, 1200);
          });
        }
        for (const btn of box.querySelectorAll("[data-rename-task]")) {
          btn.addEventListener("click", async () => {
            const task = state.tasks.find(item => item.id === btn.dataset.renameTask);
            if (!task) return;
            const name = window.prompt("请输入新的任务名称", task.name || "");
            if (name === null) return;
            const trimmed = name.trim();
            if (!trimmed) return;
            try {
              await api("POST", "/api/tasks/update", {task_id: task.id, name: trimmed});
              await loadTasks();
            } catch (err) { setLog("activeTask", err); }
          });
        }
        for (const btn of box.querySelectorAll("[data-delete-task]")) {
          btn.addEventListener("click", async () => {
            const task = state.tasks.find(item => item.id === btn.dataset.deleteTask);
            if (!task) return;
            if (!window.confirm(`确认删除任务“${task.name || task.id}”？点位不会被删除。`)) return;
            try {
              await api("DELETE", `/api/tasks?id=${encodeURIComponent(task.id)}`);
              await loadTasks();
            } catch (err) { setLog("activeTask", err); }
          });
        }
      } catch (err) {
        setLog("activeTask", err);
      } finally {
        state.loadingTasks = false;
      }
    }
    async function mainLoop() {
      const dot = $("statusDot");
      const label = $("statusText");
      try {
        const s = await fetchJson("/api/state");
        await refreshLiveMap(s.map_version);
        updateState(s);
        dot.className = "dot ok";
        label.textContent = "已连接";
        draw();
        if (activeTabName() === "tasks" && Date.now() - state.lastTasksRefreshAt > 3000) {
          loadTasks().catch(console.warn);
        }
      } catch (err) {
        dot.className = "dot warn";
        label.textContent = "等待服务";
        console.warn(err);
      } finally {
        setTimeout(mainLoop, 1500);
      }
    }
    for (const btn of document.querySelectorAll("button.tab")) {
      btn.addEventListener("click", () => {
        document.querySelectorAll("button.tab").forEach(item => item.classList.remove("active"));
        document.querySelectorAll(".panel").forEach(item => item.classList.remove("active"));
        btn.classList.add("active");
        $(`tab-${btn.dataset.tab}`).classList.add("active");
        draw();
      });
    }
    canvas.addEventListener("pointerdown", (evt) => {
      if (state.view.panMode || evt.button === 1 || evt.button === 2 || evt.shiftKey || evt.altKey) {
        evt.preventDefault();
        state.panPointer = {
          id: evt.pointerId,
          x: evt.clientX,
          y: evt.clientY,
          panX: state.view.panX,
          panY: state.view.panY
        };
        canvas.classList.add("panning");
        canvas.setPointerCapture(evt.pointerId);
        return;
      }
      const p = canvasToWorld(evt.clientX, evt.clientY);
      if (!p) return;
      evt.preventDefault();
      state.markPointer = {id: evt.pointerId, start: p, moved: false, mode: activeTabName() === "localize" ? "localize" : "mark"};
      canvas.setPointerCapture(evt.pointerId);
      if (state.markPointer.mode === "localize") {
        setLocalizeDraft(
          {x: p.x, y: p.y, yaw: currentLocalizeYaw()},
          `定位 x ${p.x.toFixed(3)} / y ${p.y.toFixed(3)} / 拖动设置朝向`
        );
      } else {
        setMarkDraft(
          {x: p.x, y: p.y, yaw: currentMarkYaw()},
          `x ${p.x.toFixed(3)} / y ${p.y.toFixed(3)} / 拖动设置朝向`
        );
      }
    });
    canvas.addEventListener("pointermove", (evt) => {
      if (state.panPointer && state.panPointer.id === evt.pointerId) {
        evt.preventDefault();
        state.view.panX = state.panPointer.panX + (evt.clientX - state.panPointer.x);
        state.view.panY = state.panPointer.panY + (evt.clientY - state.panPointer.y);
        clampView();
        draw();
        $("cursor").textContent = `地图缩放 ${Math.round(state.view.zoom * 100)}%`;
        return;
      }
      const p = canvasToWorld(evt.clientX, evt.clientY);
      if (!p) return;
      if (!state.markPointer || state.markPointer.id !== evt.pointerId) {
        $("cursor").textContent = `x ${p.x.toFixed(3)} / y ${p.y.toFixed(3)}`;
        return;
      }
      evt.preventDefault();
      const start = state.markPointer.start;
      const distance = Math.hypot(p.x - start.x, p.y - start.y);
      const yaw = distance > 0.03 ? Math.atan2(p.y - start.y, p.x - start.x) : (
        state.markPointer.mode === "localize" ? currentLocalizeYaw() : currentMarkYaw()
      );
      state.markPointer.moved = state.markPointer.moved || distance > 0.03;
      if (state.markPointer.mode === "localize") {
        setLocalizeDraft(
          {x: start.x, y: start.y, yaw},
          `定位 x ${start.x.toFixed(3)} / y ${start.y.toFixed(3)} / 朝向 ${normalizeYaw(yaw).toFixed(3)} rad`
        );
      } else {
        setMarkDraft(
          {x: start.x, y: start.y, yaw},
          `x ${start.x.toFixed(3)} / y ${start.y.toFixed(3)} / 朝向 ${normalizeYaw(yaw).toFixed(3)} rad`
        );
      }
    });
    function finishMarkPointer(evt) {
      if (state.panPointer && state.panPointer.id === evt.pointerId) {
        evt.preventDefault();
        if (canvas.hasPointerCapture(evt.pointerId)) canvas.releasePointerCapture(evt.pointerId);
        state.panPointer = null;
        canvas.classList.remove("panning");
        return;
      }
      if (!state.markPointer || state.markPointer.id !== evt.pointerId) return;
      evt.preventDefault();
      if (canvas.hasPointerCapture(evt.pointerId)) canvas.releasePointerCapture(evt.pointerId);
      const mode = state.markPointer.mode;
      const pose = mode === "localize" ? state.localizeDraft : state.markDraft;
      state.markPointer = null;
      if (pose) {
        $("cursor").textContent = `${mode === "localize" ? "待重定位" : "待保存"} x ${pose.x.toFixed(3)} / y ${pose.y.toFixed(3)} / 朝向 ${pose.yaw.toFixed(3)} rad`;
      }
    }
    canvas.addEventListener("pointerup", finishMarkPointer);
    canvas.addEventListener("pointercancel", finishMarkPointer);
    canvas.addEventListener("contextmenu", (evt) => evt.preventDefault());
    canvas.addEventListener("wheel", (evt) => {
      evt.preventDefault();
      if (!state.map) return;
      const factor = Math.exp(-evt.deltaY * 0.0012);
      setZoomAt(evt.clientX, evt.clientY, state.view.zoom * factor);
    }, {passive: false});
    $("zoomOutBtn").addEventListener("click", () => zoomBy(1 / 1.25));
    $("zoomInBtn").addEventListener("click", () => zoomBy(1.25));
    $("panModeBtn").addEventListener("click", () => {
      state.view.panMode = !state.view.panMode;
      $("panModeBtn").classList.toggle("active-tool", state.view.panMode);
      if (state.view.panMode) {
        state.followRobot = false;
        $("followRobotBtn").classList.remove("active-tool");
      }
      $("cursor").textContent = state.view.panMode ? "平移模式" : "拖拽地图取点和朝向";
    });
    $("fitMapBtn").addEventListener("click", () => {
      resetMapView(true);
    });
    $("centerRobotBtn").addEventListener("click", () => {
      const mismatch = selectedMapFloorMismatchText();
      if (mismatch) {
        $("cursor").textContent = `${mismatch}，不居中到其他楼层的实时位姿`;
        return;
      }
      const pose = freshPose();
      if (!pose) {
        $("cursor").textContent = "暂无实时机器人位姿，重定位成功且定位正常后才能居中";
        return;
      }
      centerMapOnWorld(pose.x, pose.y);
    });
    $("followRobotBtn").classList.toggle("active-tool", state.followRobot);
    $("followRobotBtn").addEventListener("click", () => {
      state.followRobot = !state.followRobot;
      $("followRobotBtn").classList.toggle("active-tool", state.followRobot);
      if (state.followRobot) {
        const mismatch = selectedMapFloorMismatchText();
        if (mismatch) {
          state.followRobot = false;
          $("followRobotBtn").classList.remove("active-tool");
          $("cursor").textContent = `${mismatch}，不能跟随其他楼层的实时位姿`;
          return;
        }
        state.view.panMode = false;
        $("panModeBtn").classList.remove("active-tool");
        const pose = freshPose();
        if (pose) centerMapOnWorld(pose.x, pose.y);
      }
    });
    $("markYaw").addEventListener("input", () => {
      const pose = state.markDraft;
      if (!pose) return;
      state.markDraft = {x: pose.x, y: pose.y, yaw: currentMarkYaw()};
      draw();
    });
    $("markXY").addEventListener("input", () => {
      const [xText, yText] = $("markXY").value.split(",");
      const x = Number(xText);
      const y = Number(yText);
      if (!Number.isFinite(x) || !Number.isFinite(y)) return;
      state.markDraft = {x, y, yaw: currentMarkYaw()};
      draw();
    });
    $("locYaw").addEventListener("input", () => {
      const pose = state.localizeDraft;
      if (!pose) return;
      state.localizeDraft = {x: pose.x, y: pose.y, yaw: currentLocalizeYaw()};
      draw();
    });
    $("locXY").addEventListener("input", () => {
      const [xText, yText] = $("locXY").value.split(",");
      const x = Number(xText);
      const y = Number(yText);
      if (!Number.isFinite(x) || !Number.isFinite(y)) return;
      state.localizeDraft = {x, y, yaw: currentLocalizeYaw()};
      draw();
    });
    $("sendInitialPoseBtn").addEventListener("click", async () => {
      try {
        if (!state.map) throw {message: "还没有固定地图，请先在地图页选择 F20 或等待默认地图加载"};
        const [xText, yText] = $("locXY").value.split(",");
        const x = Number(xText);
        const y = Number(yText);
        const yaw = Number($("locYaw").value);
        if (!Number.isFinite(x) || !Number.isFinite(y)) throw {message: "定位坐标无效，请先在地图上拖箭头"};
        state.relocalizationApiLogUntil = Date.now() + 20000;
        setLog("localizeLog", "正在发布 /initialpose，并等待开发手册 2101/1 重定位回执和原厂定位更新...");
        const payload = await api("POST", "/api/localization/initialpose", {
          x,
          y,
          z: 0,
          yaw: Number.isFinite(yaw) ? yaw : 0,
          floor: $("locFloor").value.trim()
        });
        state.relocalizationApiLogUntil = Date.now() + 12000;
        if (payload.localization_status) {
          renderLocalizationStatus({localization_status: payload.localization_status});
        }
        setLog("localizeLog", payload);
        if (payload.confirmed) {
          state.localizeDraft = null;
          try {
            updateState(await fetchJson("/api/state"));
          } catch (err) {
            console.warn(err);
          }
        }
        const manualText = payload.tcp_2101_required
          ? (payload.tcp_2101_accepted ? "收到2101回执" : "未收到2101确认回执")
          : "";
        const statusText = payload.confirmed ? "重定位成功" : "重定位失败";
        $("cursor").textContent = `${statusText}${manualText ? " / " + manualText : ""} / x ${x.toFixed(3)} / y ${y.toFixed(3)} / 朝向 ${normalizeYaw(yaw).toFixed(3)} rad`;
      } catch (err) { setLog("localizeLog", err); }
    });
    $("useRobotPoseForLocBtn").addEventListener("click", () => {
      const pose = freshPose();
      if (!pose) {
        $("cursor").textContent = "暂无实时机器人位姿，不能取旧位姿做重定位初值";
        return;
      }
      setLocalizeDraft({x: pose.x, y: pose.y, yaw: pose.yaw}, "已取当前机器人位姿");
      if (state.latest.floor) $("locFloor").value = state.latest.floor;
    });
    $("scanOverlayToggle").addEventListener("change", () => {
      state.scanOverlay = $("scanOverlayToggle").checked;
      draw();
    });
    $("checkMappingEnvBtn").addEventListener("click", async () => {
      try { setLog("mappingLog", await api("POST", "/api/mapping/check_environment", {})); }
      catch (err) { setLog("mappingLog", err); }
    });
    $("createSessionBtn").addEventListener("click", async () => {
      try {
        const payload = await api("POST", "/api/mapping/session", {
          project_name: $("projectName").value,
          building: $("buildingName").value,
          mode: $("mappingMode").value,
          floors: $("mappingFloors").value.split(",").map(v => v.trim()).filter(Boolean),
          active_floor: $("mappingActiveFloor").value.trim(),
          map_name: $("mappingMapName").value.trim()
        });
        state.sessionId = payload.session.id;
        if (!$("importName").value.trim()) $("importName").value = payload.session.map_name || "";
        setLog("mappingLog", payload);
      } catch (err) { setLog("mappingLog", err); }
    });
    $("startMappingBtn").addEventListener("click", async () => {
      try { setLog("mappingLog", await api("POST", "/api/mapping/start", {session_id: state.sessionId})); }
      catch (err) { setLog("mappingLog", err); }
    });
    $("finishMappingBtn").addEventListener("click", async () => {
      try { setLog("mappingLog", await api("POST", "/api/mapping/finish", {session_id: state.sessionId})); }
      catch (err) { setLog("mappingLog", err); }
    });
    $("importMapBtn").addEventListener("click", async () => {
      try {
        const payload = await api("POST", "/api/mapping/import_active_map", {
          session_id: state.sessionId,
          floor: $("importFloor").value.trim(),
          map_name: $("importName").value.trim()
        });
        setLog("mappingLog", payload);
        await loadMaps();
      } catch (err) { setLog("mappingLog", err); }
    });
    async function applySelectedMap() {
      const mapId = $("mapSelect").value;
      if (mapId) await loadFileMap(mapId);
      else {
        await loadFileMap("");
        state.liveMapVersion = -1;
      }
      $("cursor").textContent = mapId
        ? `已切换显示地图：${displayedMapFloor() || mapId}`
        : "已切换到实时 /map";
      try {
        const result = await api("POST", "/api/maps/select", {map_id: mapId});
        setLog("mapsLog", result);
      } catch (err) {
        setLog("mapsLog", err);
        $("cursor").textContent = `已切换前端显示；后端同步失败：${err.message || JSON.stringify(err)}`;
      }
      try {
        updateState(await fetchJson("/api/state"));
      } catch (err) {
        console.warn(err);
      }
      await loadAnnotations();
      draw();
    }
    $("selectMapBtn").addEventListener("click", async () => {
      try {
        await applySelectedMap();
      } catch (err) {
        console.warn(err);
        $("cursor").textContent = err.message || JSON.stringify(err);
      }
    });
    $("mapSelect").addEventListener("change", async () => {
      try {
        await applySelectedMap();
      } catch (err) {
        console.warn(err);
        $("cursor").textContent = err.message || JSON.stringify(err);
      }
    });
    $("reloadMapsBtn").addEventListener("click", loadMaps);
    $("markType").addEventListener("change", () => {
      const manualType = manualTypeByUiType[$("markType").value] || "task";
      $("manualPointType").value = manualType;
      syncManualDefaults(true);
    });
    $("manualPointType").addEventListener("change", () => syncManualDefaults(true));
    $("markFloor").addEventListener("input", () => updateMarkControls());
    $("saveMarkBtn").addEventListener("click", async () => {
      try {
        const blocked = markBlockedReason();
        if (blocked) throw {message: blocked};
        const [xText, yText] = $("markXY").value.split(",");
        const x = Number(xText);
        const y = Number(yText);
        if (!Number.isFinite(x) || !Number.isFinite(y)) throw {message: "点位坐标无效，请先点击地图取点"};
        const yaw = Number($("markYaw").value);
        const payload = await api("POST", "/api/annotations", {
          map_id: currentAnnotationMapId(),
          source: state.markDraftSource || "map_click",
          type: $("markType").value,
          floor: $("markFloor").value.trim(),
          label: $("markLabel").value.trim(),
          area: $("markArea").value.trim(),
          room: $("markRoom").value.trim(),
          result_file_prefix: $("markResultPrefix").value.trim(),
          pose: {
            x,
            y,
            z: 0,
            yaw: Number.isFinite(yaw) ? yaw : 0
          },
          manual_point_type: $("manualPointType").value,
          dwell_s: asNumber("markDwell", 0),
          vendor_navigation: {
            Gait: asInteger("markGait", 12),
            Speed: asInteger("markSpeed", 1),
            Manner: asInteger("markManner", 0),
            ObsMode: asInteger("markObsMode", 0),
            NavMode: asInteger("markNavMode", 1)
          }
        });
        await loadAnnotations();
        state.markDraft = null;
        state.markDraftSource = "map_click";
        draw();
        $("markLabel").value = "";
        $("markResultPrefix").value = "";
        $("cursor").textContent = `已保存 ${payload.annotation.label || payload.annotation.id}`;
      } catch (err) { $("cursor").textContent = err.message || JSON.stringify(err); }
    });
    $("useRobotPoseBtn").addEventListener("click", () => {
      const blocked = markBlockedReason();
      if (blocked) {
        $("cursor").textContent = blocked;
        updateMarkControls();
        return;
      }
      const mismatch = selectedMapFloorMismatchText();
      if (mismatch) {
        $("cursor").textContent = `${mismatch}，不能使用当前机器人位姿标到这张地图`;
        updateMarkControls();
        return;
      }
      const pose = freshPose();
      if (!pose) {
        $("cursor").textContent = "暂无实时机器人位姿，不能用旧位姿标点";
        return;
      }
      $("markXY").value = `${pose.x.toFixed(3)}, ${pose.y.toFixed(3)}`;
      $("markYaw").value = String(pose.yaw.toFixed(4));
      state.markDraft = {x: pose.x, y: pose.y, yaw: normalizeYaw(pose.yaw)};
      state.markDraftSource = "robot_pose";
      draw();
      if (state.latest.floor) $("markFloor").value = state.latest.floor;
    });
    $("createTaskBtn").addEventListener("click", async () => {
      try {
        const ids = Array.from($("taskPointList").querySelectorAll("input:checked")).map(item => item.value);
        if (!ids.length) throw {message: "当前地图还没有选中的任务点，请先在当前地图标点并勾选点位"};
        await api("POST", "/api/tasks", {
          name: $("taskName").value.trim(),
          map_id: currentAnnotationMapId(),
          annotation_ids: ids
        });
        await loadTasks();
        setLog("activeTask", "任务已生成；启动前请先复制并执行该任务卡片里的验收命令");
      } catch (err) { setLog("activeTask", err); }
    });
	    $("reloadTasksBtn").addEventListener("click", loadTasks);
	    $("frontVideoBtn").addEventListener("click", () => toggleVideo("front"));
	    $("rearVideoBtn").addEventListener("click", () => toggleVideo("rear"));
	    $("runPreflightBtn").addEventListener("click", runPreflight);
    $("refreshPreflightBtn").addEventListener("click", loadPreflight);
    $("taskRunPreflightBtn").addEventListener("click", async () => {
      document.querySelectorAll("button.tab").forEach(item => item.classList.remove("active"));
      document.querySelectorAll(".panel").forEach(item => item.classList.remove("active"));
      document.querySelector('button.tab[data-tab="preflight"]').classList.add("active");
      $("tab-preflight").classList.add("active");
      await runPreflight();
    });
    $("copyFieldSnapshotBtn").addEventListener("click", async () => {
      const btn = $("copyFieldSnapshotBtn");
      const oldText = btn.textContent;
      btn.disabled = true;
      try {
        const ok = await copyFieldSnapshot();
        btn.textContent = ok ? "快照已复制" : "复制失败";
      } catch (err) {
        setLog("activeTask", err);
        btn.textContent = "复制失败";
      } finally {
        setTimeout(() => {
          btn.disabled = false;
          btn.textContent = oldText;
        }, 1200);
      }
    });
    $("stopTaskBtn").addEventListener("click", async () => {
      const btn = $("stopTaskBtn");
      if (!(state.latest && (state.latest.active_task || state.latest.active_waypoint))) {
        btn.disabled = true;
        btn.title = "当前没有前端任务在执行";
        setLog("activeTask", "当前没有前端任务在执行，无需停止");
        return;
      }
      const oldText = btn.textContent;
      btn.disabled = true;
      btn.textContent = "停止中...";
      try {
        const payload = await api("POST", "/api/tasks/stop", {reason: "web_manual_stop"});
        setLog("activeTask", payload.message || payload.active_task || "已发送停止指令");
        await loadTasks();
      } catch (err) {
        setLog("activeTask", err);
      } finally {
        btn.disabled = false;
        btn.textContent = oldText;
      }
    });
    $("resetTaskSessionBtn").addEventListener("click", async () => {
      const btn = $("resetTaskSessionBtn");
      if (!window.confirm("确认复位导航状态？这会发送停止/零速度、清理导航会话并清代价地图。无任务时不要用它代替普通刷新。")) {
        return;
      }
      btn.disabled = true;
      const oldText = btn.textContent;
      btn.textContent = "复位中...";
      try {
        const payload = await api("POST", "/api/tasks/stop", {reason: "web_manual_reset"});
        setLog("activeTask", payload.message || payload.active_task || "已复位导航状态");
        await loadTasks();
      } catch (err) {
        setLog("activeTask", err);
      } finally {
        btn.disabled = false;
        btn.textContent = oldText;
      }
    });
    window.addEventListener("resize", resizeCanvas);
	    resizeCanvas();
	    setVideoActive(null);
	    updateMapModeUi();
    syncManualDefaults(false);
    loadMaps().then(loadAnnotations).then(loadPreflight).then(loadTasks).catch(console.warn);
    mainLoop();
