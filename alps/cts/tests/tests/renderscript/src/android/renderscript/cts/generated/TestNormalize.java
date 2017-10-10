/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Don't edit this file!  It is auto-generated by frameworks/rs/api/generate.sh.

package android.renderscript.cts;

import android.renderscript.Allocation;
import android.renderscript.RSRuntimeException;
import android.renderscript.Element;
import android.renderscript.cts.Target;

import java.util.Arrays;

public class TestNormalize extends RSBaseCompute {

    private ScriptC_TestNormalize script;
    private ScriptC_TestNormalizeRelaxed scriptRelaxed;

    @Override
    protected void setUp() throws Exception {
        super.setUp();
        script = new ScriptC_TestNormalize(mRS);
        scriptRelaxed = new ScriptC_TestNormalizeRelaxed(mRS);
    }

    public class ArgumentsFloatFloat {
        public float inV;
        public Target.Floaty out;
    }

    private void checkNormalizeFloatFloat() {
        Allocation inV = createRandomAllocation(mRS, Element.DataType.FLOAT_32, 1, 0x9460061cl, false);
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_32, 1), INPUTSIZE);
            script.forEach_testNormalizeFloatFloat(inV, out);
            verifyResultsNormalizeFloatFloat(inV, out, false);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeFloatFloat: " + e.toString());
        }
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_32, 1), INPUTSIZE);
            scriptRelaxed.forEach_testNormalizeFloatFloat(inV, out);
            verifyResultsNormalizeFloatFloat(inV, out, true);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeFloatFloat: " + e.toString());
        }
    }

    private void verifyResultsNormalizeFloatFloat(Allocation inV, Allocation out, boolean relaxed) {
        float[] arrayInV = new float[INPUTSIZE * 1];
        Arrays.fill(arrayInV, (float) 42);
        inV.copyTo(arrayInV);
        float[] arrayOut = new float[INPUTSIZE * 1];
        Arrays.fill(arrayOut, (float) 42);
        out.copyTo(arrayOut);
        StringBuilder message = new StringBuilder();
        boolean errorFound = false;
        for (int i = 0; i < INPUTSIZE; i++) {
            ArgumentsFloatFloat args = new ArgumentsFloatFloat();
            // Create the appropriate sized arrays in args
            // Fill args with the input values
            args.inV = arrayInV[i];
            Target target = new Target(Target.FunctionType.NORMAL, Target.ReturnType.FLOAT, relaxed);
            CoreMathVerifier.computeNormalize(args, target);

            // Compare the expected outputs to the actual values returned by RS.
            boolean valid = true;
            if (!args.out.couldBe(arrayOut[i])) {
                valid = false;
            }
            if (!valid) {
                if (!errorFound) {
                    errorFound = true;
                    message.append("Input inV: ");
                    appendVariableToMessage(message, arrayInV[i]);
                    message.append("\n");
                    message.append("Expected output out: ");
                    appendVariableToMessage(message, args.out);
                    message.append("\n");
                    message.append("Actual   output out: ");
                    appendVariableToMessage(message, arrayOut[i]);
                    if (!args.out.couldBe(arrayOut[i])) {
                        message.append(" FAIL");
                    }
                    message.append("\n");
                    message.append("Errors at");
                }
                message.append(" [");
                message.append(Integer.toString(i));
                message.append("]");
            }
        }
        assertFalse("Incorrect output for checkNormalizeFloatFloat" +
                (relaxed ? "_relaxed" : "") + ":\n" + message.toString(), errorFound);
    }

    public class ArgumentsFloatNFloatN {
        public float[] inV;
        public Target.Floaty[] out;
    }

    private void checkNormalizeFloat2Float2() {
        Allocation inV = createRandomAllocation(mRS, Element.DataType.FLOAT_32, 2, 0x6e066120l, false);
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_32, 2), INPUTSIZE);
            script.forEach_testNormalizeFloat2Float2(inV, out);
            verifyResultsNormalizeFloat2Float2(inV, out, false);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeFloat2Float2: " + e.toString());
        }
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_32, 2), INPUTSIZE);
            scriptRelaxed.forEach_testNormalizeFloat2Float2(inV, out);
            verifyResultsNormalizeFloat2Float2(inV, out, true);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeFloat2Float2: " + e.toString());
        }
    }

    private void verifyResultsNormalizeFloat2Float2(Allocation inV, Allocation out, boolean relaxed) {
        float[] arrayInV = new float[INPUTSIZE * 2];
        Arrays.fill(arrayInV, (float) 42);
        inV.copyTo(arrayInV);
        float[] arrayOut = new float[INPUTSIZE * 2];
        Arrays.fill(arrayOut, (float) 42);
        out.copyTo(arrayOut);
        StringBuilder message = new StringBuilder();
        boolean errorFound = false;
        for (int i = 0; i < INPUTSIZE; i++) {
            ArgumentsFloatNFloatN args = new ArgumentsFloatNFloatN();
            // Create the appropriate sized arrays in args
            args.inV = new float[2];
            args.out = new Target.Floaty[2];
            // Fill args with the input values
            for (int j = 0; j < 2 ; j++) {
                args.inV[j] = arrayInV[i * 2 + j];
            }
            Target target = new Target(Target.FunctionType.NORMAL, Target.ReturnType.FLOAT, relaxed);
            CoreMathVerifier.computeNormalize(args, target);

            // Compare the expected outputs to the actual values returned by RS.
            boolean valid = true;
            for (int j = 0; j < 2 ; j++) {
                if (!args.out[j].couldBe(arrayOut[i * 2 + j])) {
                    valid = false;
                }
            }
            if (!valid) {
                if (!errorFound) {
                    errorFound = true;
                    for (int j = 0; j < 2 ; j++) {
                        message.append("Input inV: ");
                        appendVariableToMessage(message, arrayInV[i * 2 + j]);
                        message.append("\n");
                    }
                    for (int j = 0; j < 2 ; j++) {
                        message.append("Expected output out: ");
                        appendVariableToMessage(message, args.out[j]);
                        message.append("\n");
                        message.append("Actual   output out: ");
                        appendVariableToMessage(message, arrayOut[i * 2 + j]);
                        if (!args.out[j].couldBe(arrayOut[i * 2 + j])) {
                            message.append(" FAIL");
                        }
                        message.append("\n");
                    }
                    message.append("Errors at");
                }
                message.append(" [");
                message.append(Integer.toString(i));
                message.append("]");
            }
        }
        assertFalse("Incorrect output for checkNormalizeFloat2Float2" +
                (relaxed ? "_relaxed" : "") + ":\n" + message.toString(), errorFound);
    }

    private void checkNormalizeFloat3Float3() {
        Allocation inV = createRandomAllocation(mRS, Element.DataType.FLOAT_32, 3, 0x642181fel, false);
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_32, 3), INPUTSIZE);
            script.forEach_testNormalizeFloat3Float3(inV, out);
            verifyResultsNormalizeFloat3Float3(inV, out, false);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeFloat3Float3: " + e.toString());
        }
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_32, 3), INPUTSIZE);
            scriptRelaxed.forEach_testNormalizeFloat3Float3(inV, out);
            verifyResultsNormalizeFloat3Float3(inV, out, true);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeFloat3Float3: " + e.toString());
        }
    }

    private void verifyResultsNormalizeFloat3Float3(Allocation inV, Allocation out, boolean relaxed) {
        float[] arrayInV = new float[INPUTSIZE * 4];
        Arrays.fill(arrayInV, (float) 42);
        inV.copyTo(arrayInV);
        float[] arrayOut = new float[INPUTSIZE * 4];
        Arrays.fill(arrayOut, (float) 42);
        out.copyTo(arrayOut);
        StringBuilder message = new StringBuilder();
        boolean errorFound = false;
        for (int i = 0; i < INPUTSIZE; i++) {
            ArgumentsFloatNFloatN args = new ArgumentsFloatNFloatN();
            // Create the appropriate sized arrays in args
            args.inV = new float[3];
            args.out = new Target.Floaty[3];
            // Fill args with the input values
            for (int j = 0; j < 3 ; j++) {
                args.inV[j] = arrayInV[i * 4 + j];
            }
            Target target = new Target(Target.FunctionType.NORMAL, Target.ReturnType.FLOAT, relaxed);
            CoreMathVerifier.computeNormalize(args, target);

            // Compare the expected outputs to the actual values returned by RS.
            boolean valid = true;
            for (int j = 0; j < 3 ; j++) {
                if (!args.out[j].couldBe(arrayOut[i * 4 + j])) {
                    valid = false;
                }
            }
            if (!valid) {
                if (!errorFound) {
                    errorFound = true;
                    for (int j = 0; j < 3 ; j++) {
                        message.append("Input inV: ");
                        appendVariableToMessage(message, arrayInV[i * 4 + j]);
                        message.append("\n");
                    }
                    for (int j = 0; j < 3 ; j++) {
                        message.append("Expected output out: ");
                        appendVariableToMessage(message, args.out[j]);
                        message.append("\n");
                        message.append("Actual   output out: ");
                        appendVariableToMessage(message, arrayOut[i * 4 + j]);
                        if (!args.out[j].couldBe(arrayOut[i * 4 + j])) {
                            message.append(" FAIL");
                        }
                        message.append("\n");
                    }
                    message.append("Errors at");
                }
                message.append(" [");
                message.append(Integer.toString(i));
                message.append("]");
            }
        }
        assertFalse("Incorrect output for checkNormalizeFloat3Float3" +
                (relaxed ? "_relaxed" : "") + ":\n" + message.toString(), errorFound);
    }

    private void checkNormalizeFloat4Float4() {
        Allocation inV = createRandomAllocation(mRS, Element.DataType.FLOAT_32, 4, 0x5a3ca2dcl, false);
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_32, 4), INPUTSIZE);
            script.forEach_testNormalizeFloat4Float4(inV, out);
            verifyResultsNormalizeFloat4Float4(inV, out, false);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeFloat4Float4: " + e.toString());
        }
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_32, 4), INPUTSIZE);
            scriptRelaxed.forEach_testNormalizeFloat4Float4(inV, out);
            verifyResultsNormalizeFloat4Float4(inV, out, true);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeFloat4Float4: " + e.toString());
        }
    }

    private void verifyResultsNormalizeFloat4Float4(Allocation inV, Allocation out, boolean relaxed) {
        float[] arrayInV = new float[INPUTSIZE * 4];
        Arrays.fill(arrayInV, (float) 42);
        inV.copyTo(arrayInV);
        float[] arrayOut = new float[INPUTSIZE * 4];
        Arrays.fill(arrayOut, (float) 42);
        out.copyTo(arrayOut);
        StringBuilder message = new StringBuilder();
        boolean errorFound = false;
        for (int i = 0; i < INPUTSIZE; i++) {
            ArgumentsFloatNFloatN args = new ArgumentsFloatNFloatN();
            // Create the appropriate sized arrays in args
            args.inV = new float[4];
            args.out = new Target.Floaty[4];
            // Fill args with the input values
            for (int j = 0; j < 4 ; j++) {
                args.inV[j] = arrayInV[i * 4 + j];
            }
            Target target = new Target(Target.FunctionType.NORMAL, Target.ReturnType.FLOAT, relaxed);
            CoreMathVerifier.computeNormalize(args, target);

            // Compare the expected outputs to the actual values returned by RS.
            boolean valid = true;
            for (int j = 0; j < 4 ; j++) {
                if (!args.out[j].couldBe(arrayOut[i * 4 + j])) {
                    valid = false;
                }
            }
            if (!valid) {
                if (!errorFound) {
                    errorFound = true;
                    for (int j = 0; j < 4 ; j++) {
                        message.append("Input inV: ");
                        appendVariableToMessage(message, arrayInV[i * 4 + j]);
                        message.append("\n");
                    }
                    for (int j = 0; j < 4 ; j++) {
                        message.append("Expected output out: ");
                        appendVariableToMessage(message, args.out[j]);
                        message.append("\n");
                        message.append("Actual   output out: ");
                        appendVariableToMessage(message, arrayOut[i * 4 + j]);
                        if (!args.out[j].couldBe(arrayOut[i * 4 + j])) {
                            message.append(" FAIL");
                        }
                        message.append("\n");
                    }
                    message.append("Errors at");
                }
                message.append(" [");
                message.append(Integer.toString(i));
                message.append("]");
            }
        }
        assertFalse("Incorrect output for checkNormalizeFloat4Float4" +
                (relaxed ? "_relaxed" : "") + ":\n" + message.toString(), errorFound);
    }

    public class ArgumentsHalfHalf {
        public short inV;
        public double inVDouble;
        public Target.Floaty out;
    }

    private void checkNormalizeHalfHalf() {
        Allocation inV = createRandomAllocation(mRS, Element.DataType.FLOAT_16, 1, 0x2f4d2e6l, false);
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_16, 1), INPUTSIZE);
            script.forEach_testNormalizeHalfHalf(inV, out);
            verifyResultsNormalizeHalfHalf(inV, out, false);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeHalfHalf: " + e.toString());
        }
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_16, 1), INPUTSIZE);
            scriptRelaxed.forEach_testNormalizeHalfHalf(inV, out);
            verifyResultsNormalizeHalfHalf(inV, out, true);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeHalfHalf: " + e.toString());
        }
    }

    private void verifyResultsNormalizeHalfHalf(Allocation inV, Allocation out, boolean relaxed) {
        short[] arrayInV = new short[INPUTSIZE * 1];
        Arrays.fill(arrayInV, (short) 42);
        inV.copyTo(arrayInV);
        short[] arrayOut = new short[INPUTSIZE * 1];
        Arrays.fill(arrayOut, (short) 42);
        out.copyTo(arrayOut);
        StringBuilder message = new StringBuilder();
        boolean errorFound = false;
        for (int i = 0; i < INPUTSIZE; i++) {
            ArgumentsHalfHalf args = new ArgumentsHalfHalf();
            // Create the appropriate sized arrays in args
            // Fill args with the input values
            args.inV = arrayInV[i];
            args.inVDouble = Float16Utils.convertFloat16ToDouble(args.inV);
            Target target = new Target(Target.FunctionType.NORMAL, Target.ReturnType.HALF, relaxed);
            CoreMathVerifier.computeNormalize(args, target);

            // Compare the expected outputs to the actual values returned by RS.
            boolean valid = true;
            if (!args.out.couldBe(Float16Utils.convertFloat16ToDouble(arrayOut[i]))) {
                valid = false;
            }
            if (!valid) {
                if (!errorFound) {
                    errorFound = true;
                    message.append("Input inV: ");
                    appendVariableToMessage(message, arrayInV[i]);
                    message.append("\n");
                    message.append("Expected output out: ");
                    appendVariableToMessage(message, args.out);
                    message.append("\n");
                    message.append("Actual   output out: ");
                    appendVariableToMessage(message, arrayOut[i]);
                    message.append("\n");
                    message.append("Actual   output out (in double): ");
                    appendVariableToMessage(message, Float16Utils.convertFloat16ToDouble(arrayOut[i]));
                    if (!args.out.couldBe(Float16Utils.convertFloat16ToDouble(arrayOut[i]))) {
                        message.append(" FAIL");
                    }
                    message.append("\n");
                    message.append("Errors at");
                }
                message.append(" [");
                message.append(Integer.toString(i));
                message.append("]");
            }
        }
        assertFalse("Incorrect output for checkNormalizeHalfHalf" +
                (relaxed ? "_relaxed" : "") + ":\n" + message.toString(), errorFound);
    }

    public class ArgumentsHalfNHalfN {
        public short[] inV;
        public double[] inVDouble;
        public Target.Floaty[] out;
    }

    private void checkNormalizeHalf2Half2() {
        Allocation inV = createRandomAllocation(mRS, Element.DataType.FLOAT_16, 2, 0x17e4d58l, false);
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_16, 2), INPUTSIZE);
            script.forEach_testNormalizeHalf2Half2(inV, out);
            verifyResultsNormalizeHalf2Half2(inV, out, false);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeHalf2Half2: " + e.toString());
        }
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_16, 2), INPUTSIZE);
            scriptRelaxed.forEach_testNormalizeHalf2Half2(inV, out);
            verifyResultsNormalizeHalf2Half2(inV, out, true);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeHalf2Half2: " + e.toString());
        }
    }

    private void verifyResultsNormalizeHalf2Half2(Allocation inV, Allocation out, boolean relaxed) {
        short[] arrayInV = new short[INPUTSIZE * 2];
        Arrays.fill(arrayInV, (short) 42);
        inV.copyTo(arrayInV);
        short[] arrayOut = new short[INPUTSIZE * 2];
        Arrays.fill(arrayOut, (short) 42);
        out.copyTo(arrayOut);
        StringBuilder message = new StringBuilder();
        boolean errorFound = false;
        for (int i = 0; i < INPUTSIZE; i++) {
            ArgumentsHalfNHalfN args = new ArgumentsHalfNHalfN();
            // Create the appropriate sized arrays in args
            args.inV = new short[2];
            args.inVDouble = new double[2];
            args.out = new Target.Floaty[2];
            // Fill args with the input values
            for (int j = 0; j < 2 ; j++) {
                args.inV[j] = arrayInV[i * 2 + j];
                args.inVDouble[j] = Float16Utils.convertFloat16ToDouble(args.inV[j]);
            }
            Target target = new Target(Target.FunctionType.NORMAL, Target.ReturnType.HALF, relaxed);
            CoreMathVerifier.computeNormalize(args, target);

            // Compare the expected outputs to the actual values returned by RS.
            boolean valid = true;
            for (int j = 0; j < 2 ; j++) {
                if (!args.out[j].couldBe(Float16Utils.convertFloat16ToDouble(arrayOut[i * 2 + j]))) {
                    valid = false;
                }
            }
            if (!valid) {
                if (!errorFound) {
                    errorFound = true;
                    for (int j = 0; j < 2 ; j++) {
                        message.append("Input inV: ");
                        appendVariableToMessage(message, arrayInV[i * 2 + j]);
                        message.append("\n");
                    }
                    for (int j = 0; j < 2 ; j++) {
                        message.append("Expected output out: ");
                        appendVariableToMessage(message, args.out[j]);
                        message.append("\n");
                        message.append("Actual   output out: ");
                        appendVariableToMessage(message, arrayOut[i * 2 + j]);
                        message.append("\n");
                        message.append("Actual   output out (in double): ");
                        appendVariableToMessage(message, Float16Utils.convertFloat16ToDouble(arrayOut[i * 2 + j]));
                        if (!args.out[j].couldBe(Float16Utils.convertFloat16ToDouble(arrayOut[i * 2 + j]))) {
                            message.append(" FAIL");
                        }
                        message.append("\n");
                    }
                    message.append("Errors at");
                }
                message.append(" [");
                message.append(Integer.toString(i));
                message.append("]");
            }
        }
        assertFalse("Incorrect output for checkNormalizeHalf2Half2" +
                (relaxed ? "_relaxed" : "") + ":\n" + message.toString(), errorFound);
    }

    private void checkNormalizeHalf3Half3() {
        Allocation inV = createRandomAllocation(mRS, Element.DataType.FLOAT_16, 3, 0x6086124cl, false);
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_16, 3), INPUTSIZE);
            script.forEach_testNormalizeHalf3Half3(inV, out);
            verifyResultsNormalizeHalf3Half3(inV, out, false);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeHalf3Half3: " + e.toString());
        }
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_16, 3), INPUTSIZE);
            scriptRelaxed.forEach_testNormalizeHalf3Half3(inV, out);
            verifyResultsNormalizeHalf3Half3(inV, out, true);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeHalf3Half3: " + e.toString());
        }
    }

    private void verifyResultsNormalizeHalf3Half3(Allocation inV, Allocation out, boolean relaxed) {
        short[] arrayInV = new short[INPUTSIZE * 4];
        Arrays.fill(arrayInV, (short) 42);
        inV.copyTo(arrayInV);
        short[] arrayOut = new short[INPUTSIZE * 4];
        Arrays.fill(arrayOut, (short) 42);
        out.copyTo(arrayOut);
        StringBuilder message = new StringBuilder();
        boolean errorFound = false;
        for (int i = 0; i < INPUTSIZE; i++) {
            ArgumentsHalfNHalfN args = new ArgumentsHalfNHalfN();
            // Create the appropriate sized arrays in args
            args.inV = new short[3];
            args.inVDouble = new double[3];
            args.out = new Target.Floaty[3];
            // Fill args with the input values
            for (int j = 0; j < 3 ; j++) {
                args.inV[j] = arrayInV[i * 4 + j];
                args.inVDouble[j] = Float16Utils.convertFloat16ToDouble(args.inV[j]);
            }
            Target target = new Target(Target.FunctionType.NORMAL, Target.ReturnType.HALF, relaxed);
            CoreMathVerifier.computeNormalize(args, target);

            // Compare the expected outputs to the actual values returned by RS.
            boolean valid = true;
            for (int j = 0; j < 3 ; j++) {
                if (!args.out[j].couldBe(Float16Utils.convertFloat16ToDouble(arrayOut[i * 4 + j]))) {
                    valid = false;
                }
            }
            if (!valid) {
                if (!errorFound) {
                    errorFound = true;
                    for (int j = 0; j < 3 ; j++) {
                        message.append("Input inV: ");
                        appendVariableToMessage(message, arrayInV[i * 4 + j]);
                        message.append("\n");
                    }
                    for (int j = 0; j < 3 ; j++) {
                        message.append("Expected output out: ");
                        appendVariableToMessage(message, args.out[j]);
                        message.append("\n");
                        message.append("Actual   output out: ");
                        appendVariableToMessage(message, arrayOut[i * 4 + j]);
                        message.append("\n");
                        message.append("Actual   output out (in double): ");
                        appendVariableToMessage(message, Float16Utils.convertFloat16ToDouble(arrayOut[i * 4 + j]));
                        if (!args.out[j].couldBe(Float16Utils.convertFloat16ToDouble(arrayOut[i * 4 + j]))) {
                            message.append(" FAIL");
                        }
                        message.append("\n");
                    }
                    message.append("Errors at");
                }
                message.append(" [");
                message.append(Integer.toString(i));
                message.append("]");
            }
        }
        assertFalse("Incorrect output for checkNormalizeHalf3Half3" +
                (relaxed ? "_relaxed" : "") + ":\n" + message.toString(), errorFound);
    }

    private void checkNormalizeHalf4Half4() {
        Allocation inV = createRandomAllocation(mRS, Element.DataType.FLOAT_16, 4, 0xbf8dd740l, false);
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_16, 4), INPUTSIZE);
            script.forEach_testNormalizeHalf4Half4(inV, out);
            verifyResultsNormalizeHalf4Half4(inV, out, false);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeHalf4Half4: " + e.toString());
        }
        try {
            Allocation out = Allocation.createSized(mRS, getElement(mRS, Element.DataType.FLOAT_16, 4), INPUTSIZE);
            scriptRelaxed.forEach_testNormalizeHalf4Half4(inV, out);
            verifyResultsNormalizeHalf4Half4(inV, out, true);
        } catch (Exception e) {
            throw new RSRuntimeException("RenderScript. Can't invoke forEach_testNormalizeHalf4Half4: " + e.toString());
        }
    }

    private void verifyResultsNormalizeHalf4Half4(Allocation inV, Allocation out, boolean relaxed) {
        short[] arrayInV = new short[INPUTSIZE * 4];
        Arrays.fill(arrayInV, (short) 42);
        inV.copyTo(arrayInV);
        short[] arrayOut = new short[INPUTSIZE * 4];
        Arrays.fill(arrayOut, (short) 42);
        out.copyTo(arrayOut);
        StringBuilder message = new StringBuilder();
        boolean errorFound = false;
        for (int i = 0; i < INPUTSIZE; i++) {
            ArgumentsHalfNHalfN args = new ArgumentsHalfNHalfN();
            // Create the appropriate sized arrays in args
            args.inV = new short[4];
            args.inVDouble = new double[4];
            args.out = new Target.Floaty[4];
            // Fill args with the input values
            for (int j = 0; j < 4 ; j++) {
                args.inV[j] = arrayInV[i * 4 + j];
                args.inVDouble[j] = Float16Utils.convertFloat16ToDouble(args.inV[j]);
            }
            Target target = new Target(Target.FunctionType.NORMAL, Target.ReturnType.HALF, relaxed);
            CoreMathVerifier.computeNormalize(args, target);

            // Compare the expected outputs to the actual values returned by RS.
            boolean valid = true;
            for (int j = 0; j < 4 ; j++) {
                if (!args.out[j].couldBe(Float16Utils.convertFloat16ToDouble(arrayOut[i * 4 + j]))) {
                    valid = false;
                }
            }
            if (!valid) {
                if (!errorFound) {
                    errorFound = true;
                    for (int j = 0; j < 4 ; j++) {
                        message.append("Input inV: ");
                        appendVariableToMessage(message, arrayInV[i * 4 + j]);
                        message.append("\n");
                    }
                    for (int j = 0; j < 4 ; j++) {
                        message.append("Expected output out: ");
                        appendVariableToMessage(message, args.out[j]);
                        message.append("\n");
                        message.append("Actual   output out: ");
                        appendVariableToMessage(message, arrayOut[i * 4 + j]);
                        message.append("\n");
                        message.append("Actual   output out (in double): ");
                        appendVariableToMessage(message, Float16Utils.convertFloat16ToDouble(arrayOut[i * 4 + j]));
                        if (!args.out[j].couldBe(Float16Utils.convertFloat16ToDouble(arrayOut[i * 4 + j]))) {
                            message.append(" FAIL");
                        }
                        message.append("\n");
                    }
                    message.append("Errors at");
                }
                message.append(" [");
                message.append(Integer.toString(i));
                message.append("]");
            }
        }
        assertFalse("Incorrect output for checkNormalizeHalf4Half4" +
                (relaxed ? "_relaxed" : "") + ":\n" + message.toString(), errorFound);
    }

    public void testNormalize() {
        checkNormalizeFloatFloat();
        checkNormalizeFloat2Float2();
        checkNormalizeFloat3Float3();
        checkNormalizeFloat4Float4();
        checkNormalizeHalfHalf();
        checkNormalizeHalf2Half2();
        checkNormalizeHalf3Half3();
        checkNormalizeHalf4Half4();
    }
}
