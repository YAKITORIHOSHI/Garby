package com.thesis.garby

import android.graphics.Bitmap
import android.graphics.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.ColorFilter
import androidx.compose.ui.graphics.Shape
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import com.android.tools.screenshot.PreviewTest
import com.thesis.garby.ui.theme.GarbyTheme

@Composable
private fun NativeLauncher(resource: Int) {
    val context = LocalContext.current
    val bitmap = remember(resource) {
        val drawable = context.getDrawable(resource)!!
        Bitmap.createBitmap(96, 96, Bitmap.Config.ARGB_8888).also {
            drawable.setBounds(0, 0, 96, 96)
            drawable.draw(Canvas(it))
        }.asImageBitmap()
    }
    Image(bitmap, null, modifier = Modifier.size(96.dp))
}

@Composable
private fun MaskedIcon(shape: Shape, size: Dp, themed: Boolean = false) {
    Box(
        Modifier.size(size).clip(shape).background(if (themed) Color(0xFFE5D7F5) else Color(0xFF7B0A0A)),
        contentAlignment = Alignment.Center
    ) {
        Image(
            painterResource(R.drawable.ic_launcher_foreground), null,
            colorFilter = if (themed) ColorFilter.tint(Color(0xFF342347)) else null,
            modifier = Modifier.requiredSize(size * 1.5f)
        )
    }
}

@PreviewTest
@Preview(device = "spec:width=400dp,height=470dp,dpi=160", widthDp = 400, heightDp = 470)
@Composable
fun LauncherIconRender() {
    GarbyTheme {
        Column(
            Modifier.fillMaxSize().background(Color(0xFF170E13)).padding(24.dp),
            verticalArrangement = Arrangement.spacedBy(20.dp)
        ) {
            Text("Android adaptive icon", color = Color.White, style = MaterialTheme.typography.titleMedium)
            Row(horizontalArrangement = Arrangement.spacedBy(32.dp)) {
                Column(horizontalAlignment = Alignment.CenterHorizontally) {
                    NativeLauncher(R.mipmap.ic_launcher_before)
                    Text("Before", color = Color(0xFFCCBCC2))
                }
                Column(horizontalAlignment = Alignment.CenterHorizontally) {
                    NativeLauncher(R.mipmap.ic_launcher)
                    Text("GARBY", color = Color.White)
                }
            }
            Text("Launcher masks · 72 dp", color = Color.White)
            Row(horizontalArrangement = Arrangement.spacedBy(16.dp)) {
                MaskedIcon(CircleShape, 72.dp)
                MaskedIcon(RoundedCornerShape(22.dp), 72.dp)
                MaskedIcon(RoundedCornerShape(topStart = 36.dp, topEnd = 36.dp, bottomEnd = 6.dp, bottomStart = 36.dp), 72.dp)
                MaskedIcon(CircleShape, 72.dp, themed = true)
            }
            Text("Small icons · 48 dp", color = Color.White)
            Row(horizontalArrangement = Arrangement.spacedBy(18.dp)) {
                MaskedIcon(CircleShape, 48.dp)
                MaskedIcon(RoundedCornerShape(14.dp), 48.dp)
                MaskedIcon(CircleShape, 48.dp, themed = true)
            }
        }
    }
}
