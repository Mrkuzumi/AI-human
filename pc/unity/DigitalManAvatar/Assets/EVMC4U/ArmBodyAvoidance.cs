using UnityEngine;

namespace EVMC4U
{
    // Project the requested two-bone chain around the torso; never switch to a preset pose.
    public sealed class ArmBodyAvoidance
    {
        sealed class Arm
        {
            public Transform upper, lower, hand;
            public readonly Transform[] tips = new Transform[5];
            public readonly Vector3[] offsets = new Vector3[5];
            public Quaternion inputUpper, inputLower;
            public Quaternion outputUpper, outputLower;
            public Vector3 previousElbow;
            public bool initialized;
            public float lateral, depthSide = 1f;
        }

        // shortcut: coat/hip sections measured from model.prefab, refit after replacing this model/outfit.
        static readonly float[] levels = { -1.15f, -0.90f, -0.30f, 0f, 0.65f, 1.15f, 1.50f, 1.85f, 2.10f, 2.40f };
        static readonly float[] widths = { 0f, 0.70f, 1.00f, 1.02f, 1.04f, 1.04f, 0.88f, 0.68f, 0.38f, 0f };
        static readonly float[] depths = { 0f, 0.60f, 0.88f, 0.92f, 0.88f, 0.85f, 0.78f, 0.70f, 0.55f, 0f };
        Animator model;
        Transform hips;
        Arm left, right;
        Vector3 origin;
        Quaternion frame, inverse;
        float width;

        public void Apply(Animator animator, bool applyLeft = true, bool applyRight = true, float deltaTime = -1f)
        {
            if (!animator || !animator.isHuman) return;
            if (model != animator)
            {
                model = animator;
                hips = model.GetBoneTransform(HumanBodyBones.Hips);
                left = Bind(true);
                right = Bind(false);
            }
            if (!hips || left == null || right == null) return;
            origin = hips.position;
            Vector3 center = (left.upper.position + right.upper.position) * 0.5f;
            Vector3 up = (center - origin).normalized;
            Vector3 across = right.upper.position - left.upper.position;
            width = across.magnitude;
            if (width < 0.001f) return;
            Vector3 forward = Vector3.Cross(across, up).normalized;
            if (Vector3.Dot(forward, hips.forward) < 0f) forward = -forward;
            if (forward.sqrMagnitude < 0.5f) return;
            frame = Quaternion.LookRotation(forward, up);
            inverse = Quaternion.Inverse(frame);
            float dt = deltaTime > 0f ? deltaTime : Application.isPlaying ? Time.deltaTime : 1f / 30f;
            dt = Mathf.Clamp(dt, 1f / 240f, 0.1f);
            if (applyLeft) Solve(left, dt); else left.initialized = false;
            if (applyRight) Solve(right, dt); else right.initialized = false;
        }

        Arm Bind(bool isLeft)
        {
            var arm = new Arm {
                upper = model.GetBoneTransform(isLeft ? HumanBodyBones.LeftUpperArm : HumanBodyBones.RightUpperArm),
                lower = model.GetBoneTransform(isLeft ? HumanBodyBones.LeftLowerArm : HumanBodyBones.RightLowerArm),
                hand = model.GetBoneTransform(isLeft ? HumanBodyBones.LeftHand : HumanBodyBones.RightHand)
            };
            if (!arm.upper || !arm.lower || !arm.hand) return null;
            HumanBodyBones[] tips = { HumanBodyBones.LeftThumbDistal, HumanBodyBones.LeftIndexDistal,
                HumanBodyBones.LeftMiddleDistal, HumanBodyBones.LeftRingDistal, HumanBodyBones.LeftLittleDistal };
            for (int i = 0; i < tips.Length; i++)
                arm.tips[i] = model.GetBoneTransform(isLeft ? tips[i] :
                    tips[i] + (HumanBodyBones.RightThumbDistal - HumanBodyBones.LeftThumbDistal));
            return arm;
        }

        Vector3 Local(Vector3 point) => inverse * (point - origin);

        void Solve(Arm arm, float dt)
        {
            Quaternion palm = arm.hand.rotation;
            if (arm.initialized)
            {
                float alpha = 1f - Mathf.Exp(-18f * dt);
                arm.upper.localRotation = Quaternion.RotateTowards(arm.inputUpper,
                    Quaternion.Slerp(arm.inputUpper, arm.upper.localRotation, alpha), 240f * dt);
                arm.lower.localRotation = Quaternion.RotateTowards(arm.inputLower,
                    Quaternion.Slerp(arm.inputLower, arm.lower.localRotation, alpha), 240f * dt);
                arm.hand.rotation = palm;
            }
            // Filter the requested pose separately: feeding collision corrections back into it sticks the arm.
            arm.inputUpper = arm.upper.localRotation;
            arm.inputLower = arm.lower.localRotation;
            Vector3 shoulder = Local(arm.upper.position), elbow = Local(arm.lower.position), wrist = Local(arm.hand.position);
            arm.lateral = Mathf.Sign(shoulder.x);
            if (!arm.initialized) arm.depthSide = wrist.z < -width * 0.9f ? -1f : 1f;
            // Changing depth is free outside the sides or above the torso.
            if (Mathf.Abs(wrist.x) > width * 1.3f && Mathf.Abs(elbow.x) > width ||
                wrist.y > width * levels[levels.Length - 1])
                if (Mathf.Abs(wrist.z) > width * 0.2f) arm.depthSide = Mathf.Sign(wrist.z);
            for (int i = 0; i < arm.tips.Length; i++)
            {
                Transform tip = arm.tips[i];
                Vector3 end = tip ? tip.position + (tip.position - tip.parent.position) * 0.7f : arm.hand.position;
                arm.offsets[i] = inverse * (end - arm.hand.position);
            }
            float upperLength = Vector3.Distance(shoulder, elbow), lowerLength = Vector3.Distance(elbow, wrist);
            if (upperLength < 0.0001f || lowerLength < 0.0001f) return;
            float inward = (shoulder.x - wrist.x) * arm.lateral / width;
            float reach = Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(0.05f, 0.45f, inward));
            Vector3 target = wrist, hint = elbow;
            if (arm.initialized) hint = Vector3.Lerp(elbow, arm.previousElbow, 0.15f);
            for (int iteration = 0; iteration < 32; iteration++)
            {
                TwoBone(shoulder, target, hint, upperLength, lowerLength, arm.lateral, out elbow, out wrist);
                Vector3 correction = Vector3.zero;
                float amount = 0f, leverage = 1f;
                for (int i = 0; i <= 16; i++)
                {
                    float t = i / 16f;
                    Select(Push(Vector3.Lerp(elbow, wrist, t), arm, reach), 2f / (1f + t),
                        ref correction, ref amount, ref leverage);
                }
                for (int i = 1; i <= 8; i++)
                {
                    float t = i / 8f;
                    Vector3 point = Vector3.Lerp(shoulder, elbow, t);
                    // Adjacent upper-arm/shoulder skin is attached. Protect the upper arm if it crosses the trunk.
                    float attachment = Mathf.SmoothStep(0f, 1f,
                        Mathf.InverseLerp(Mathf.Abs(shoulder.x) * 0.95f, Mathf.Abs(shoulder.x) * 0.25f,
                            point.x * arm.lateral));
                    Select(Push(point, arm, reach) * attachment, 2f,
                        ref correction, ref amount, ref leverage);
                }
                for (int i = 0; i < arm.offsets.Length; i++)
                    for (int j = 1; j <= 4; j++)
                        Select(Push(wrist + arm.offsets[i] * (j / 4f), arm, reach), 1f,
                            ref correction, ref amount, ref leverage);
                if (amount < width * 0.002f) break;
                target = wrist + correction * leverage;
                hint = elbow + correction;
            }
            arm.upper.rotation = Quaternion.FromToRotation(arm.lower.position - arm.upper.position,
                frame * (elbow - shoulder)) * arm.upper.rotation;
            arm.lower.rotation = Quaternion.FromToRotation(arm.hand.position - arm.lower.position,
                frame * (wrist - elbow)) * arm.lower.rotation;
            if (arm.initialized)
            {
                float alpha = 1f - Mathf.Exp(-24f * dt);
                arm.upper.localRotation = Quaternion.RotateTowards(arm.outputUpper,
                    Quaternion.Slerp(arm.outputUpper, arm.upper.localRotation, alpha), 165f * dt);
                arm.lower.localRotation = Quaternion.RotateTowards(arm.outputLower,
                    Quaternion.Slerp(arm.outputLower, arm.lower.localRotation, alpha), 165f * dt);
            }
            arm.hand.rotation = palm;
            arm.outputUpper = arm.upper.localRotation;
            arm.outputLower = arm.lower.localRotation;
            arm.previousElbow = Local(arm.lower.position);
            arm.initialized = true;
        }

        static void Select(Vector3 delta, float factor, ref Vector3 correction, ref float amount, ref float leverage)
        {
            float length = delta.magnitude;
            if (length <= amount) return;
            amount = length;
            correction = delta;
            leverage = factor;
        }

        Vector3 Push(Vector3 point, Arm arm, float reach)
        {
            float height = point.y / width;
            if (height <= levels[0] || height >= levels[levels.Length - 1]) return Vector3.zero;
            int i = 1;
            while (height > levels[i]) i++;
            float t = Mathf.InverseLerp(levels[i - 1], levels[i], height);
            float rx = width * Mathf.Lerp(widths[i - 1], widths[i], t);
            float rz = width * Mathf.Lerp(depths[i - 1], depths[i], t);
            if (rx < 0.0001f || rz < 0.0001f) return Vector3.zero;
            float x = point.x / rx, z = point.z / rz;
            float q = x * x + z * z;
            if (q >= 1.002f) return Vector3.zero;
            Vector3 side = new Vector3(arm.lateral * rx * Mathf.Sqrt(Mathf.Max(0f, 1.004f - z * z)), point.y, point.z);
            Vector3 front = new Vector3(point.x, point.y,
                arm.depthSide * rz * Mathf.Sqrt(Mathf.Max(0f, 1.004f - x * x)));
            Vector3 direction = Vector3.Lerp(side - point, front - point, reach);
            float a = direction.x * direction.x / (rx * rx) + direction.z * direction.z / (rz * rz);
            if (a < 1e-10f) return Vector3.zero;
            float b = 2f * (point.x * direction.x / (rx * rx) + point.z * direction.z / (rz * rz));
            float scale = (-b + Mathf.Sqrt(Mathf.Max(0f, b * b - 4f * a * (q - 1.004f)))) / (2f * a);
            return direction * scale;
        }

        static void TwoBone(Vector3 shoulder, Vector3 target, Vector3 hint, float upperLength, float lowerLength,
            float lateral, out Vector3 elbow, out Vector3 wrist)
        {
            Vector3 delta = target - shoulder;
            float distance = Mathf.Clamp(delta.magnitude, Mathf.Abs(upperLength - lowerLength) + 0.00001f,
                upperLength + lowerLength);
            Vector3 direction = delta.sqrMagnitude > 1e-10f ? delta.normalized : Vector3.down;
            wrist = shoulder + direction * distance;
            float along = (upperLength * upperLength - lowerLength * lowerLength + distance * distance) / (2f * distance);
            float height = Mathf.Sqrt(Mathf.Max(0f, upperLength * upperLength - along * along));
            Vector3 pole = Vector3.ProjectOnPlane(hint - shoulder, direction);
            if (pole.sqrMagnitude < 1e-8f) pole = Vector3.ProjectOnPlane(Vector3.right * lateral, direction);
            if (pole.sqrMagnitude < 1e-8f) pole = Vector3.ProjectOnPlane(Vector3.forward, direction);
            elbow = shoulder + direction * along + pole.normalized * height;
        }
    }
}
